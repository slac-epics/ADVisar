TOP=../..
include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES
DB_INSTALLS += ADVisar.db

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
# Make sure the *.substitutions file must be the same name as *.db file
#DB += xxx.db

##for iocAdmin > than R2-0-0
#DB_INSTALLS += $(IOCADMIN)/db/iocAdmin.db 
#DB_INSTALLS += $(IOCADMIN)/db/access.db #(only if used for devices) 
#DB_INSTALLS += $(IOCADMIN)/db/baseSecurity.acf  

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_TEMPLATE = <templatename>
#ARCHIVE += staubli_ioc.archive
#ARCHIVE += staubli_archive.archive

#----------------------------------------------------
# Create and install (or just install)
# Autosave request files, templates, substitutions like this
#
#REQ += staubli_ioc.req
#REQ += staubli_autosave.req

# Nothing to build for autosave, just install from $AUTOSAVE/db
#DB_INSTALLS += $(AUTOSAVE)/db/save_restoreStatus.db

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

