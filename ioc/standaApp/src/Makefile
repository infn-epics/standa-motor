TOP=../..

include $(TOP)/configure/CONFIG

# Driver library
LIBRARY_IOC += USMC
DBD += USMCSupport.dbd
INC += USMCDriver.h
USMC_SRCS += USMCDriver.cpp
USMC_LIBS += motor asyn
USMC_LIBS += $(EPICS_BASE_IOC_LIBS)
USMC_SYS_LIBS += usb-1.0
USR_INCLUDES += -I/usr/include/libusb-1.0

# IOC application
PROD_IOC = standa
DBD += standa.dbd
standa_DBD += base.dbd
standa_DBD += asyn.dbd
standa_DBD += motorSupport.dbd
standa_DBD += USMCSupport.dbd

standa_SRCS += standa_registerRecordDeviceDriver.cpp
standa_SRCS_DEFAULT += standaMain.cpp
standa_LIBS += USMC motor asyn
standa_LIBS += $(EPICS_BASE_IOC_LIBS)
standa_SYS_LIBS += usb-1.0

include $(TOP)/configure/RULES
