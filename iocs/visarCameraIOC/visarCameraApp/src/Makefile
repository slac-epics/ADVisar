TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = visarCameraApp
PROD_IOC += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd

$(PROD_NAME)_DBD += visarCameraSupport.dbd



# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS += visarCamera

$(PROD_NAME)_LIBS += asyn
$(PROD_NAME)_LIBS += stream
$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)


include $(ADCORE)/ADApp/commonDriverMakefile
$(PROD_NAME)_DBD += stream.dbd                 # Must come after include $(ADCORE)/ADApp/commonDriverMakefile because it needs base.dbd


ifneq '$(T_A)' 'RTEMS-beatnik'
PROD_SYS_LIBS_DEFAULT += pcre
#SHRLIB_DEPLIB_DIRS += $(PCRE_LIB)
endif


#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE



