/*
 * EPICS asyn motor driver for Standa 8SMC1-USBhF stepper motor controllers.
 * See USMCDriver.h.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <asynOctetSyncIO.h>

#include "USMCDriver.h"

#include <epicsExport.h>

static const char *driverName = "USMCDriver";

#define REQ_SET_CURRENT_POSITION 0x01
#define REQ_STOP_STEP_MOTOR      0x07
#define REQ_GO_TO                0x80
#define REQ_SET_MODE             0x81
#define REQ_GET_STATE            0x82
#define REQ_SET_PARAMETERS       0x83

#define BRIDGE_TIMEOUT 2.0   // s, covers the bridge's own 1 s USB timeout
#define MAX_LINE 512

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

// Split a list separated by commas and/or spaces
static std::vector<std::string> splitList(const char *list)
{
    std::vector<std::string> out;
    char *copy = epicsStrDup(list);
    char *save = NULL;
    for (char *tok = epicsStrtok_r(copy, ", ", &save); tok; tok = epicsStrtok_r(NULL, ", ", &save))
        out.push_back(tok);
    free(copy);
    return out;
}

static bool endsWith(const std::string &s, const std::string &tail)
{
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0;
}

/* ------------------------------------------------------------------ controller */

USMCController::USMCController(const char *portName, const char *bridgePort, const char *serials,
                               double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, (int)splitList(serials).size(), NUM_USMC_PARAMS,
                          asynOctetMask, asynOctetMask,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,   // autoconnect
                          0, 0),
      limitsEnabled_(true), tr1ActiveHigh_(false), tr2ActiveHigh_(false),
      bridgePort_(bridgePort), pasynUserBridge_(NULL)
{
    static const char *functionName = "USMCController";

    createParam(USMC_TEMPERATURE_STRING, asynParamFloat64, &temperature_);
    createParam(USMC_VOLTAGE_STRING,     asynParamFloat64, &voltage_);
    createParam(USMC_SERIAL_STRING,      asynParamOctet,   &serial_);
    createParam(USMC_CONNECTED_STRING,   asynParamInt32,   &connected_);

    if (pasynOctetSyncIO->connect(bridgePort, 0, &pasynUserBridge_, NULL)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: cannot connect to bridge port %s\n",
                  driverName, functionName, bridgePort);
        pasynUserBridge_ = NULL;
    } else {
        pasynOctetSyncIO->setInputEos(pasynUserBridge_, "\n", 1);
        pasynOctetSyncIO->setOutputEos(pasynUserBridge_, "\n", 1);
    }

    std::vector<std::string> list = splitList(serials);
    for (size_t axis = 0; axis < list.size(); axis++)
        new USMCAxis(this, (int)axis, list[axis].c_str());

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

void USMCController::report(FILE *fp, int level)
{
    fprintf(fp, "Standa 8SMC1-USBhF controller %s, bridge %s, %d axes\n",
            portName, bridgePort_.c_str(), numAxes_);
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
        if (axis && !axis->device_.empty()) axis->setMode(axis->powerOn_);
    }
    unlock();
}

// One request line to the bridge, one reply line back: "OK [data]" or "ERR message".
asynStatus USMCController::transact(const char *cmd, std::string &reply)
{
    char in[MAX_LINE];
    size_t nwrite, nread;
    int eom;

    if (!pasynUserBridge_) return asynError;
    asynStatus status = pasynOctetSyncIO->writeRead(pasynUserBridge_, cmd, strlen(cmd),
                                                    in, sizeof(in) - 1, BRIDGE_TIMEOUT,
                                                    &nwrite, &nread, &eom);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: bridge %s: '%s' failed: %s\n",
                  driverName, bridgePort_.c_str(), cmd, pasynUserBridge_->errorMessage);
        return status;
    }
    in[nread] = '\0';
    if (strncmp(in, "OK", 2) != 0 || (in[2] != '\0' && in[2] != ' ')) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: bridge %s: '%s' -> '%s'\n",
                  driverName, bridgePort_.c_str(), cmd, in);
        return asynError;
    }
    reply = in[2] ? in + 3 : "";
    return asynSuccess;
}

/* ------------------------------------------------------------------ axis */

USMCAxis::USMCAxis(USMCController *pC, int axisNo, const char *serial)
    : asynMotorAxis(pC, axisNo), pC_(pC), serial_(serial),
      version_(0), accelTimeMs_(-1.0), powerOn_(true)
{
    setStringParam(pC_->serial_, serial);
    setIntegerParam(pC_->connected_, 0);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);  // CNEN switches motor power
    connect();
    callParamCallbacks();
}

void USMCAxis::report(FILE *fp, int level)
{
    fprintf(fp, "  axis %d: serial %s, %s, firmware %x\n", axisNo_, serial_.c_str(),
            device_.empty() ? "NOT connected" : device_.c_str(), version_);
    asynMotorAxis::report(fp, level);
}

// Ask the bridge for its controllers ("OK serial:bcdDevice ...") and pick the one
// whose serial ends with serial_; then power the motor.
asynStatus USMCAxis::connect()
{
    static const char *functionName = "connect";
    std::string reply;

    if (!device_.empty()) return asynSuccess;
    if (pC_->transact("LIST", reply)) return asynError;

    std::vector<std::string> devs = splitList(reply.c_str());
    for (size_t i = 0; i < devs.size(); i++) {
        size_t colon = devs[i].find(':');
        std::string sn = devs[i].substr(0, colon);
        if (endsWith(sn, serial_)) {
            device_ = sn;
            version_ = colon == std::string::npos ? 0 : strtoul(devs[i].c_str() + colon + 1, NULL, 16);
            break;
        }
    }
    if (device_.empty()) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %d: no 8SMC1 with serial %s on bridge\n",
                  driverName, functionName, axisNo_, serial_.c_str());
        setIntegerParam(pC_->motorStatusCommsError_, 1);
        return asynError;
    }
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: axis %d: serial %s firmware %x\n",
              driverName, functionName, axisNo_, device_.c_str(), version_);
    setIntegerParam(pC_->connected_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 0);
    accelTimeMs_ = -1.0;
    return setMode(powerOn_);
}

void USMCAxis::disconnect()
{
    device_.clear();
    setIntegerParam(pC_->connected_, 0);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
}

asynStatus USMCAxis::ctrlIn(unsigned req, unsigned char *buf, unsigned len, const char *what)
{
    char cmd[64];
    std::string reply;

    if (device_.empty()) return asynError;
    epicsSnprintf(cmd, sizeof(cmd), "IN %s %02X 0 0 %u", device_.c_str(), req, len);
    if (pC_->transact(cmd, reply) || reply.size() != 2 * len) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: axis %d (%s): %s failed\n",
                  driverName, axisNo_, serial_.c_str(), what);
        disconnect();
        return asynError;
    }
    for (unsigned i = 0; i < len; i++)
        buf[i] = (unsigned char)strtoul(reply.substr(2 * i, 2).c_str(), NULL, 16);
    return asynSuccess;
}

asynStatus USMCAxis::ctrlOut(unsigned req, unsigned value, unsigned index, const unsigned char *buf,
                             unsigned len, const char *what)
{
    char cmd[MAX_LINE];
    std::string reply;

    if (device_.empty()) return asynError;
    int n = epicsSnprintf(cmd, sizeof(cmd), "OUT %s %02X %X %X ", device_.c_str(), req,
                          value & 0xFFFF, index & 0xFFFF);
    for (unsigned i = 0; i < len && n + 3 < (int)sizeof(cmd); i++)
        n += epicsSnprintf(cmd + n, sizeof(cmd) - n, "%02X", buf[i]);
    if (len == 0) cmd[n - 1] = '\0';
    if (pC_->transact(cmd, reply)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: axis %d (%s): %s failed\n",
                  driverName, axisNo_, serial_.c_str(), what);
        disconnect();
        return asynError;
    }
    return asynSuccess;
}

asynStatus USMCAxis::getState(USMCState *st)
{
    unsigned char b[11];
    asynStatus status = ctrlIn(REQ_GET_STATE, b, sizeof(b), "GET_STATE");
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
    return ctrlOut(REQ_GO_TO, raw >> 16, raw & 0xFFFF, data, 3, "GO_TO");
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
    asynStatus status = ctrlOut(REQ_SET_MODE, p[0] << 8 | p[1], p[2] << 8 | p[3], p + 4, 3, "SET_MODE");
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
    asynStatus status = ctrlOut(REQ_SET_PARAMETERS, p[0] << 8 | p[1], p[3] << 8 | p[2], p + 4, 53,
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
    return ctrlOut(REQ_STOP_STEP_MOTOR, 0, 0, NULL, 0, "STOP");
}

// The firmware keeps set positions in multiples of 32 (4 full steps, one phase cycle).
asynStatus USMCAxis::setPosition(double position)
{
    asynStatus status;
    if ((status = connect())) return status;
    uint32_t raw = ((uint32_t)(int)floor(position + 0.5) * 8u) & 0xFFFFFFE0u;
    return ctrlOut(REQ_SET_CURRENT_POSITION, raw >> 16, raw & 0xFFFF, NULL, 0, "SET_CURRENT_POSITION");
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

extern "C" int USMCCreateController(const char *portName, const char *bridgePort, const char *serials,
                                    int movingPollPeriodMs, int idlePollPeriodMs)
{
    if (!portName || !bridgePort || !serials || !*serials) {
        printf("usage: USMCCreateController(port, bridgeAsynPort, \"serial1 serial2 ...\", movingPollMs, idlePollMs)\n");
        return asynError;
    }
    new USMCController(portName, bridgePort, serials,
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
static const iocshArg createArg1 = {"Bridge asyn IP port", iocshArgString};
static const iocshArg createArg2 = {"Serial numbers (one per axis)", iocshArgString};
static const iocshArg createArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg createArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const createArgs[] = {&createArg0, &createArg1, &createArg2, &createArg3, &createArg4};
static const iocshFuncDef createDef = {"USMCCreateController", 5, createArgs};
static void createCall(const iocshArgBuf *args)
{
    USMCCreateController(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival);
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
