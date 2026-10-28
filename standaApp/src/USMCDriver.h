/*
 * EPICS asyn motor driver for Standa 8SMC1-USBhF stepper motor controllers.
 *
 * Each 8SMC1 is a separate USB device driving one motor, so one EPICS
 * controller (asyn port) groups several USB devices, one per axis, selected
 * by serial number.  The USB devices are reached through usmc_server.py, a
 * bridge that forwards vendor control transfers received over TCP (one line
 * per transfer, see python/usmc_server.py).  The 8SMC1 protocol itself is
 * implemented here; python/usmc.py is the reference.
 *
 * Units: positions in 1/8 full steps, velocities in (1/8 steps)/s.
 */
#ifndef USMC_DRIVER_H
#define USMC_DRIVER_H

#include <string>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define USMC_TEMPERATURE_STRING "USMC_TEMPERATURE"
#define USMC_VOLTAGE_STRING     "USMC_VOLTAGE"
#define USMC_SERIAL_STRING      "USMC_SERIAL"
#define USMC_CONNECTED_STRING   "USMC_CONNECTED"

struct USMCState {
    int    curPos;      // 1/8 steps
    double temp;        // C
    double voltage;     // V
    bool   loft, fullPower, cwccw, power, fullSpeed, afterReset;
    bool   run, syncIn, syncOut, rotTr, rotTrErr, emReset, trailer1, trailer2;
    bool   usbPower, working;
};

class epicsShareClass USMCAxis : public asynMotorAxis
{
public:
    USMCAxis(class USMCController *pC, int axisNo, const char *serial);
    void report(FILE *fp, int level);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
    asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setPosition(double position);
    asynStatus setClosedLoop(bool closedLoop);

private:
    asynStatus connect();
    void disconnect();
    asynStatus ctrlIn(unsigned req, unsigned char *buf, unsigned len, const char *what);
    asynStatus ctrlOut(unsigned req, unsigned value, unsigned index, const unsigned char *buf,
                       unsigned len, const char *what);
    asynStatus getState(USMCState *st);
    asynStatus goTo(int position, double speed);
    asynStatus setMode(bool powerOn);
    asynStatus setParameters(double accelTimeMs);

    USMCController *pC_;
    std::string serial_;    // as configured (may be the trailing digits only)
    std::string device_;    // full 16-char serial reported by the bridge, empty if not connected
    unsigned version_;
    double accelTimeMs_;    // last value sent, <0 if never
    bool powerOn_;

    friend class USMCController;
};

class epicsShareClass USMCController : public asynMotorController
{
public:
    USMCController(const char *portName, const char *bridgePort, const char *serials,
                   double movingPollPeriod, double idlePollPeriod);
    void report(FILE *fp, int level);
    USMCAxis *getAxis(asynUser *pasynUser);
    USMCAxis *getAxis(int axisNo);
    void configLimits(bool enable, bool tr1ActiveHigh, bool tr2ActiveHigh);

    // Limit switch configuration applied with every SET_MODE
    bool limitsEnabled_;
    bool tr1ActiveHigh_, tr2ActiveHigh_;

protected:
    int temperature_;
#define FIRST_USMC_PARAM temperature_
    int voltage_;
    int serial_;
    int connected_;
#define NUM_USMC_PARAMS 4

private:
    asynStatus transact(const char *cmd, std::string &reply);

    std::string bridgePort_;
    asynUser *pasynUserBridge_;

    friend class USMCAxis;
};

#endif
