TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build an ioc application
# Use PROD_IOC_RTEMS for RTEMS only
# Use PROD_IOC_Linux for Linux only
PROD_IOC = ADVisar

# ADVisar.dbd will be created and installed
DBD += ADVisar.dbd

# ADVisar.dbd will be made up from these files:
ADVisar_DBD += base.dbd
#ADVisar_DBD += iocAdmin.dbd
#ADVisar_DBD += autosaveSupport.dbd
ADVisar_DBD += asyn.dbd
ADVisar_DBD += asynRecord.dbd
ADVisar_DBD += drvAsynSerialPort.dbd
ADVisar_DBD += drvAsynIPPort.dbd
ADVisar_DBD += stream.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
ADVisar_SRCS += ADVisar_registerRecordDeviceDriver.cpp
ADVisar_SRCS_DEFAULT += ADVisarMain.cpp

# Add locally compiled object code
#ADVisar_SRCS += dbSubExample.c

#ADVisar_LIBS += devIocStats
#ADVisar_LIBS += autosave
ADVisar_LIBS += asyn
ADVisar_LIBS += stream
ADVisar_LIBS += $(EPICS_BASE_IOC_LIBS)

ifneq '$(T_A)' 'RTEMS-beatnik'
PROD_SYS_LIBS_DEFAULT += pcre
SHRLIB_DEPLIB_DIRS += $(PCRE_LIB)
endif

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


