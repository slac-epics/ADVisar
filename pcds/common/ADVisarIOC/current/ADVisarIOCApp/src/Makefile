TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build an ioc application
# Use PROD_IOC_RTEMS for RTEMS only
# Use PROD_IOC_Linux for Linux only
PROD_IOC = ADVisarIOC

# ADVisarIOC.dbd will be created and installed
DBD += ADVisarIOC.dbd

# ADVisarIOC.dbd will be made up from these files:
ADVisarIOC_DBD += base.dbd
ADVisarIOC_DBD += iocAdmin.dbd
ADVisarIOC_DBD += autosaveSupport.dbd
ADVisarIOC_DBD += asyn.dbd
ADVisarIOC_DBD += asynRecord.dbd
ADVisarIOC_DBD += drvAsynSerialPort.dbd
ADVisarIOC_DBD += drvAsynIPPort.dbd
ADVisarIOC_DBD += stream.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
ADVisarIOC_SRCS += ADVisarIOC_registerRecordDeviceDriver.cpp
ADVisarIOC_SRCS_DEFAULT += ADVisarIOCMain.cpp

# Add locally compiled object code
#ADVisarIOC_SRCS += dbSubExample.c

ADVisarIOC_LIBS += devIocStats
ADVisarIOC_LIBS += autosave
ADVisarIOC_LIBS += asyn
ADVisarIOC_LIBS += stream
ADVisarIOC_LIBS += $(EPICS_BASE_IOC_LIBS)

ifneq '$(T_A)' 'RTEMS-beatnik'
PROD_SYS_LIBS_DEFAULT += pcre
SHRLIB_DEPLIB_DIRS += $(PCRE_LIB)
endif

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

