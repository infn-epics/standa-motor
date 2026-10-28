#!../../bin/linux-aarch64/standa
# IOC for Standa 8SMC1-USBhF stepper motor controllers
#
# Environment (defaults below):
#   PREFIX      PV prefix
#   USMC_SERIALS  space or comma separated controller serials, one per axis (m1, m2, ...)

< envPaths

epicsEnvSet("PREFIX",       "$(PREFIX=STANDA:)")
epicsEnvSet("USMC_SERIALS", "$(USMC_SERIALS=6183 5912 6098 6140)")
epicsEnvSet("PORT",         "USMC1")

cd "$(TOP)"
dbLoadDatabase "dbd/standa.dbd"
standa_registerRecordDeviceDriver pdbbase

# USMCCreateController(port, serials, movingPollMs, idlePollMs)
USMCCreateController("$(PORT)", "$(USMC_SERIALS)", 100, 1000)
# USMCConfigLimits(port, enable, limit1ActiveHigh, limit2ActiveHigh)
USMCConfigLimits("$(PORT)", 1, 0, 0)

dbLoadTemplate("iocBoot/iocStanda/motor.substitutions", "P=$(PREFIX),PORT=$(PORT)")

cd "$(TOP)/iocBoot/$(IOC)"
iocInit
