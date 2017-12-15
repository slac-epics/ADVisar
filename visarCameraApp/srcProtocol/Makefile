T_A=$(EPICS_HOST_ARCH)
TOP=../..
include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#----------------------------------------------------

COPY_FILES += visar.proto
COPY_SUBDIR = $(TOP)/protocol

DB_INSTALLS += visar.proto

include $(TOP)/configure/RULES
include $(TOP)/configure/RULES.copy
#----------------------------------------
#  ADD RULES AFTER THIS LINE

