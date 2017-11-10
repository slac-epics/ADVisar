TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
# DB += ADVisar.db
DB += ADVisar.template
DB += ADVisar.substitutions

#----------------------------------------------------
# Create and install (or just install)
# Autosave request files, templates, substitutions like this
# REQ += autosaveADVisar.req
REQ += ADVisar.tpl-req

#----------------------------------------------------
# Create and install (or just install)
# Archive files, templates, substitutions like this
# ARCHIVE += archiveADVisar.archive
ARCHIVE += ADVisar.tpl-arch

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

