TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build a module library
# Use LIBRARY_IOC_RTEMS for RTEMS only
# Use LIBRARY_IOC_Linux for Linux only
LIBRARY_IOC = ADVisar

# ADVisar.dbd will be made up from these files:
DBD += ADVisar.dbd

# Add any header files here which should be installed
INC	+=	ADVisar.h

# Add any html documentation here
# HTMLS	+=	index.html
# HTMLS +=	ADVisar.html
HTMLS_DIR	= .

# Add any source files here
# Note: No lib will be built if no sources are listed here
LIBSRCS += ADVisar.cpp

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

