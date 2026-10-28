/*
 * EPICS asyn motor driver for Standa 8SMC1-USBhF stepper motor controllers.
 * See USMCDriver.h.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <asynOctetSyncIO.h>

#include "USMCDriver.h"

#include <epicsExport.h>

static const char *driverName = "USMCDriver";

#define USMC_VID 0x10C4
#define USMC_PID 0x0230

#define REQ_SET_CURRENT_POSITION 0x01
#define REQ_STOP_STEP_MOTOR      0x07
#define REQ_GO_TO                0x80
#define REQ_SET_MODE             0x81
#define REQ_GET_STATE            0x82
#define REQ_SET_PARAMETERS       0x83
#define REQ_GET_SERIAL           0xC9

#define RT_IN  (LIBUSB_ENDPOINT_IN  | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define RT_OUT (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)

#define USB_TIMEOUT_MS 1000

// Far target used for jogging (raw position = 8 * position must fit in int32)
#define JOG_DISTANCE (1 << 27)

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void putBE16(unsigned char *p, unsigned v) { p[0] = (v >> 8) & 0xFF; p[1] = v & 0xFF; }

// Timer reload for GO_TO speeds (USMC_Start conversion)
static unsigned speedToPeriod(double speed)
{
    return (unsigned)(65536.0 - 1000000.0 / clampd(speed, 16.0, 5000.0) + 0.5);
}

// Timer reload for parameter-packet speeds (MinP, BTOxP)
static unsigned buttonPeriod(double speed)
{
    return (unsigned)(65536.0 - 125000.0 / clampd(speed, 2.0, 625.0) + 0.5);
}

static unsigned timeoutTicks(double ms) { return (unsigned)(clampd(ms, 1.0, 9961.0) / 0.152 + 0.5); }

// Split a list of serials separated by commas and/or spaces
static std::vector<std::string> splitSerials(const char *serials)
{
    std::vector<std::string> out;
    char *list = epicsStrDup(serials);
    char *save = NULL;
    for (char *tok = epicsStrtok_r(list, ", ", &save); tok; tok = epicsStrtok_r(NULL, ", ", &save))
        out.push_back(tok);
    free(list);
    return out;
}

/* ------------------------------------------------------------------ controller */

USMCController::USMCController(const char *portName, const char *serials,
                               double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, (int)splitSerials(serials).size(),
                          NUM_USMC_PARAMS,
                          asynOctetMask, asynOctetMask,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,   // autoconnect
                          0, 0),
      limitsEnabled_(true), tr1ActiveHigh_(false), tr2ActiveHigh_(false), ctx_(NULL)
{
    static const char *functionName = "USMCController";

    createParam(USMC_TEMPERATURE_STRING, asynParamFloat64, &temperature_);
    createParam(USMC_VOLTAGE_STRING,     asynParamFloat64, &voltage_);
    createParam(USMC_SERIAL_STRING,      asynParamOctet,   &serial_);
    createParam(USMC_CONNECTED_STRING,   asynParamInt32,   &connected_);

    int rc = libusb_init(&ctx_);
    if (rc) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: libusb_init: %s\n",
                  driverName, functionName, libusb_strerror((libusb_error)rc));
        ctx_ = NULL;
    }

    std::vector<std::string> list = splitSerials(serials);
    for (size_t axis = 0; axis < list.size(); axis++)
        new USMCAxis(this, (int)axis, list[axis].c_str());

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

USMCController::~USMCController()
{
    if (ctx_) libusb_exit(ctx_);
}

void USMCController::report(FILE *fp, int level)
{
    fprintf(fp, "Standa 8SMC1-USBhF controller %s, %d axes\n", portName, numAxes_);
    asynMotorController::report(fp, level);
}

USMCAxis *USMCController::getAxis(asynUser *pasynUser)
{
    return static_cast<USMCAxis *>(asynMotorController::getAxis(pasynUser));
}

USMCAxis *USMCController::getAxis(int axisNo)
{
    return static_cast<USMCAxis *>(asynMotorController::getAxis(axisNo));
}

void USMCController::configLimits(bool enable, bool tr1ActiveHigh, bool tr2ActiveHigh)
{
    lock();
    limitsEnabled_ = enable;
    tr1ActiveHigh_ = tr1ActiveHigh;
    tr2ActiveHigh_ = tr2ActiveHigh;
    for (int i = 0; i < numAxes_; i++) {
        USMCAxis *axis = getAxis(i);
        if (axis && axis->handle_) axis->setMode(axis->powerOn_);
    }
    unlock();
}

/* ------------------------------------------------------------------ axis */

USMCAxis::USMCAxis(USMCController *pC, int axisNo, const char *serial)
    : asynMotorAxis(pC, axisNo), pC_(pC), serial_(serial), handle_(NULL),
      version_(0), accelTimeMs_(-1.0), powerOn_(true)
{
    setStringParam(pC_->serial_, serial);
    setIntegerParam(pC_->connected_, 0);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);  // CNEN switches motor power
    connect();
    callParamCallbacks();
}

USMCAxis::~USMCAxis()
{
    disconnect();
}

void USMCAxis::report(FILE *fp, int level)
{
    fprintf(fp, "  axis %d: serial %s, %s, firmware %x\n", axisNo_, serial_.c_str(),
            handle_ ? "connected" : "NOT connected", version_);
    asynMotorAxis::report(fp, level);
}

// Find the USB device whose 16-char serial ends with serial_, open it and power the motor.
asynStatus USMCAxis::connect()
{
    static const char *functionName = "connect";
    libusb_device **list;

    if (handle_) return asynSuccess;
    if (!pC_->ctx_) return asynError;

    ssize_t n = libusb_get_device_list(pC_->ctx_, &list);
    for (ssize_t i = 0; i < n && !handle_; i++) {
        libusb_device_descriptor desc;
        libusb_device_handle *h;
        if (libusb_get_device_descriptor(list[i], &desc) || desc.idVendor != USMC_VID ||
            desc.idProduct != USMC_PID || libusb_open(list[i], &h))
            continue;
        // cp210x is sometimes force-bound via new_id; it cannot drive this device.
        if (libusb_kernel_driver_active(h, 0) == 1)
            libusb_detach_kernel_driver(h, 0);
        unsigned char buf[17] = {0};
        int rc = libusb_control_transfer(h, RT_IN, REQ_GET_SERIAL, 0, 0, buf, 16, USB_TIMEOUT_MS);
        const char *sn = (const char *)buf;
        size_t len = strlen(sn);
        if (rc == 16 && len >= serial_.size() &&
            strcmp(sn + len - serial_.size(), serial_.c_str()) == 0) {
            handle_ = h;
            version_ = desc.bcdDevice;
        } else {
            libusb_close(h);
        }
    }
    if (n >= 0) libusb_free_device_list(list, 1);

    if (!handle_) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %d: no 8SMC1 with serial %s\n",
                  driverName, functionName, axisNo_, serial_.c_str());
        setIntegerParam(pC_->motorStatusCommsError_, 1);
        return asynError;
    }
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: axis %d: serial %s firmware %x\n",
              driverName, functionName, axisNo_, serial_.c_str(), version_);
    setIntegerParam(pC_->connected_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 0);
    accelTimeMs_ = -1.0;
    return setMode(powerOn_);
}

void USMCAxis::disconnect()
{
    if (handle_) libusb_close(handle_);
    handle_ = NULL;
    setIntegerParam(pC_->connected_, 0);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
}

int USMCAxis::ctrlIn(uint8_t req, unsigned char *buf, uint16_t len)
{
    if (!handle_) return LIBUSB_ERROR_NO_DEVICE;
    int rc = libusb_control_transfer(handle_, RT_IN, req, 0, 0, buf, len, USB_TIMEOUT_MS);
    return (rc >= 0 && rc != len) ? LIBUSB_ERROR_IO : rc;
}

int USMCAxis::ctrlOut(uint8_t req, uint16_t value, uint16_t index, unsigned char *buf, uint16_t len)
{
    if (!handle_) return LIBUSB_ERROR_NO_DEVICE;
    return libusb_control_transfer(handle_, RT_OUT, req, value, index, buf, len, USB_TIMEOUT_MS);
}

asynStatus USMCAxis::check(int rc, const char *what)
{
    if (rc >= 0) return asynSuccess;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: axis %d (%s): %s failed: %s\n",
              driverName, axisNo_, serial_.c_str(), what, libusb_strerror((libusb_error)rc));
    if (rc == LIBUSB_ERROR_NO_DEVICE || rc == LIBUSB_ERROR_PIPE || rc == LIBUSB_ERROR_IO)
        disconnect();
    return asynError;
}

asynStatus USMCAxis::getState(USMCState *st)
{
    unsigned char b[11];
    asynStatus status = check(ctrlIn(REQ_GET_STATE, b, sizeof(b)), "GET_STATE");
    if (status) return status;

    int32_t raw = (int32_t)(b[0] | b[1] << 8 | b[2] << 16 | (uint32_t)b[3] << 24);
    unsigned temp = b[4] | b[5] << 8;
    unsigned volt = b[9] | b[10] << 8;
    st->curPos     = raw / 8;
    st->loft       = b[6] & 0x04;
    st->fullPower  = b[6] & 0x08;
    st->cwccw      = b[6] & 0x10;
    st->power      = b[6] & 0x20;
    st->fullSpeed  = b[6] & 0x40;
    st->afterReset = b[6] & 0x80;
    st->run        = b[7] & 0x01;
    st->syncIn     = b[7] & 0x02;
    st->syncOut    = b[7] & 0x04;
    st->rotTr      = b[7] & 0x08;
    st->rotTrErr   = b[7] & 0x10;
    st->emReset    = b[7] & 0x20;
    st->trailer1   = b[7] & 0x40;
    st->trailer2   = b[7] & 0x80;
    st->usbPower   = b[8] & 0x01;
    st->working    = b[8] & 0x80;
    if (version_ < 0x2400) {
        double t = temp * 3.3 / 65536.0;
        t = t * 10.0 / (5.0 - t);
        st->temp = 1.0 / (1.0 / 298.0 + log(t / 10.0) / 3950.0) - 273.0;
    } else {
        st->temp = temp * 3.3 * 100.0 / 65536.0 - 50.0;
    }
    st->voltage = volt / 65536.0 * 3.3 * 20.0;
    if (st->voltage < 5.0) st->voltage = 0.0;
    return asynSuccess;
}

// GO_TO with 1/8 microstepping and slow start/stop (acceleration from SET_PARAMETERS).
asynStatus USMCAxis::goTo(int position, double speed)
{
    uint32_t raw = (uint32_t)position * 8u;
    unsigned period = speedToPeriod(speed);
    unsigned char data[3];
    putBE16(data, period);
    data[2] = 0x03 /* M1|M2: 1/8 step */ | 0x10 /* SLSTRT */;
    return check(ctrlOut(REQ_GO_TO, raw >> 16, raw & 0xFFFF, data, 3), "GO_TO");
}

// SET_MODE: the controller cannot report its mode, so every call sends all of it
// (USMCDLL defaults: current reduction, sync out, limit switches per controller config).
asynStatus USMCAxis::setMode(bool powerOn)
{
    unsigned char p[7] = {0};
    p[0] = 0x02 /* PReg */ | (powerOn ? 0 : 0x04 /* ResetD */) |
           (pC_->tr1ActiveHigh_ ? 0x10 : 0) | (pC_->tr2ActiveHigh_ ? 0x20 : 0);
    p[1] = (pC_->limitsEnabled_ ? 0x03 /* Tr1En|Tr2En */ : 0) | 0x08 /* RotTrOp */;
    p[2] = 0x01 /* SyncOUTEn */ | 0x04 /* SyncINOp */;
    uint32_t syncCount = 4;
    p[3] = syncCount >> 24; p[4] = syncCount >> 16; p[5] = syncCount >> 8; p[6] = syncCount;
    asynStatus status = check(ctrlOut(REQ_SET_MODE, p[0] << 8 | p[1], p[2] << 8 | p[3], p + 4, 3),
                              "SET_MODE");
    if (status == asynSuccess) powerOn_ = powerOn;
    return status;
}

// SET_PARAMETERS with USMCDLL defaults except the acceleration/deceleration time.
asynStatus USMCAxis::setParameters(double accelTimeMs)
{
    unsigned char p[57] = {0};
    unsigned delay = (unsigned)clampd(floor(accelTimeMs / 98.0 + 0.5), 1, 15);
    unsigned refInTimeout = timeoutTicks(100.0);           // PTimeout, ms
    p[0] = delay;                                          // DELAY1 (accel)
    p[1] = delay;                                          // DELAY2 (decel)
    p[2] = refInTimeout & 0xFF; p[3] = refInTimeout >> 8;  // little endian, unlike the rest
    unsigned char *q = p + 4;
    for (int i = 0; i < 6; i++, q += 2) putBE16(q, timeoutTicks(500.0));  // BTIMEOUT1-4, R, D
    putBE16(q, buttonPeriod(500.0)); q += 2;                // MINPERIOD
    putBE16(q, buttonPeriod(200.0)); q += 2;                // BTO1P
    putBE16(q, buttonPeriod(300.0)); q += 2;                // BTO2P
    putBE16(q, buttonPeriod(400.0)); q += 2;                // BTO3P
    putBE16(q, buttonPeriod(500.0)); q += 2;                // BTO4P
    putBE16(q, 32 * 64); q += 2;                            // MAX_LOFT
    q += 4;                                                 // STARTPOS = 0
    putBE16(q, 200 * 64); q += 2;                           // RTDelta
    putBE16(q, 15 * 64); q += 2;                            // RTMinError
    putBE16(q, (unsigned)((70.0 + 50.0) / 330.0 * 65536.0 + 0.5)); q += 2;  // MaxTemp 70 C
    *q++ = 1;                                               // SynOUTP
    putBE16(q, (unsigned)(65536.0 - 125000.0 / 500.0 + 0.5)); q += 2;      // LoftPeriod
    *q++ = 10;                                              // EncVSCP (EncMult 2.5 * 4)
    if (version_ < 0x2400) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: axis %d: firmware %x < 2400, parameters not sent\n", driverName, axisNo_, version_);
        return asynError;
    }
    asynStatus status = check(ctrlOut(REQ_SET_PARAMETERS, p[0] << 8 | p[1], p[3] << 8 | p[2], p + 4, 53),
                              "SET_PARAMETERS");
    if (status == asynSuccess) accelTimeMs_ = delay * 98.0;
    return status;
}

asynStatus USMCAxis::move(double position, int relative, double minVelocity, double maxVelocity,
                          double acceleration)
{
    USMCState st;
    asynStatus status;
    if ((status = connect())) return status;

    if (acceleration > 0 && maxVelocity > 0) {
        double accelTimeMs = maxVelocity / acceleration * 1000.0;
        if (fabs(floor(accelTimeMs / 98.0 + 0.5) * 98.0 - accelTimeMs_) > 1.0 &&
            (status = setParameters(accelTimeMs)))
            return status;
    }
    if (!powerOn_ && (status = setMode(true))) return status;

    double target = position;
    if (relative) {
        if ((status = getState(&st))) return status;
        target += st.curPos;
    }
    target = clampd(floor(target + 0.5), -JOG_DISTANCE, JOG_DISTANCE);
    status = goTo((int)target, maxVelocity);
    if (status == asynSuccess) {
        // RUN is raised ~50 ms after GO_TO; report moving now so the record does not see a stale done.
        setIntegerParam(pC_->motorStatusDone_, 0);
        setIntegerParam(pC_->motorStatusMoving_, 1);
        callParamCallbacks();
    }
    return status;
}

asynStatus USMCAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
    double target = maxVelocity >= 0 ? JOG_DISTANCE : -JOG_DISTANCE;
    return move(target, 0, minVelocity, fabs(maxVelocity), acceleration);
}

asynStatus USMCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: axis %d: homing not supported (no home switch input)\n", driverName, axisNo_);
    return asynError;
}

asynStatus USMCAxis::stop(double acceleration)
{
    return check(ctrlOut(REQ_STOP_STEP_MOTOR, 0, 0, NULL, 0), "STOP");
}

// The firmware keeps set positions in multiples of 32 (4 full steps, one phase cycle).
asynStatus USMCAxis::setPosition(double position)
{
    asynStatus status;
    if ((status = connect())) return status;
    uint32_t raw = ((uint32_t)(int)floor(position + 0.5) * 8u) & 0xFFFFFFE0u;
    return check(ctrlOut(REQ_SET_CURRENT_POSITION, raw >> 16, raw & 0xFFFF, NULL, 0),
                 "SET_CURRENT_POSITION");
}

asynStatus USMCAxis::setClosedLoop(bool closedLoop)
{
    asynStatus status;
    if ((status = connect())) return status;
    return setMode(closedLoop);
}

asynStatus USMCAxis::poll(bool *moving)
{
    USMCState st;
    *moving = false;

    if (connect() || getState(&st)) {
        setIntegerParam(pC_->motorStatusProblem_, 1);
        callParamCallbacks();
        return asynError;
    }
    *moving = st.run;
    setDoubleParam(pC_->motorPosition_, st.curPos);
    setDoubleParam(pC_->motorEncoderPosition_, st.curPos);
    setIntegerParam(pC_->motorStatusDone_, !st.run);
    setIntegerParam(pC_->motorStatusMoving_, st.run);
    setIntegerParam(pC_->motorStatusDirection_, !st.cwccw);
    setIntegerParam(pC_->motorStatusLowLimit_, st.trailer1);
    setIntegerParam(pC_->motorStatusHighLimit_, st.trailer2);
    setIntegerParam(pC_->motorStatusPowerOn_, st.power);
    setIntegerParam(pC_->motorStatusProblem_, !st.working || st.emReset || st.rotTrErr);
    setIntegerParam(pC_->motorStatusCommsError_, 0);
    setDoubleParam(pC_->temperature_, st.temp);
    setDoubleParam(pC_->voltage_, st.voltage);
    powerOn_ = st.power;
    callParamCallbacks();
    return asynSuccess;
}

/* ------------------------------------------------------------------ iocsh */

extern "C" int USMCCreateController(const char *portName, const char *serials,
                                    int movingPollPeriodMs, int idlePollPeriodMs)
{
    if (!portName || !serials || !*serials) {
        printf("usage: USMCCreateController(port, \"serial1,serial2,...\", movingPollMs, idlePollMs)\n");
        return asynError;
    }
    new USMCController(portName, serials,
                       (movingPollPeriodMs > 0 ? movingPollPeriodMs : 100) / 1000.0,
                       (idlePollPeriodMs > 0 ? idlePollPeriodMs : 1000) / 1000.0);
    return asynSuccess;
}

// Limit switch configuration: enable, and the active level of each switch (0: 0 V, 1: +3/5 V).
// Applied to all axes of the controller from the next SET_MODE (connect, CNEN).
extern "C" int USMCConfigLimits(const char *portName, int enable, int tr1ActiveHigh, int tr2ActiveHigh)
{
    USMCController *pC = (USMCController *)findAsynPortDriver(portName);
    if (!pC) {
        printf("USMCConfigLimits: port %s not found\n", portName);
        return asynError;
    }
    pC->configLimits(enable, tr1ActiveHigh, tr2ActiveHigh);
    return asynSuccess;
}

static const iocshArg createArg0 = {"Port name", iocshArgString};
static const iocshArg createArg1 = {"Serial numbers (comma separated, one per axis)", iocshArgString};
static const iocshArg createArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg createArg3 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const createArgs[] = {&createArg0, &createArg1, &createArg2, &createArg3};
static const iocshFuncDef createDef = {"USMCCreateController", 4, createArgs};
static void createCall(const iocshArgBuf *args)
{
    USMCCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

static const iocshArg limitsArg0 = {"Port name", iocshArgString};
static const iocshArg limitsArg1 = {"Enable limit switches", iocshArgInt};
static const iocshArg limitsArg2 = {"Limit 1 active high", iocshArgInt};
static const iocshArg limitsArg3 = {"Limit 2 active high", iocshArgInt};
static const iocshArg *const limitsArgs[] = {&limitsArg0, &limitsArg1, &limitsArg2, &limitsArg3};
static const iocshFuncDef limitsDef = {"USMCConfigLimits", 4, limitsArgs};
static void limitsCall(const iocshArgBuf *args)
{
    USMCConfigLimits(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}

static void USMCRegister(void)
{
    iocshRegister(&createDef, createCall);
    iocshRegister(&limitsDef, limitsCall);
}

extern "C" {
epicsExportRegistrar(USMCRegister);
}
