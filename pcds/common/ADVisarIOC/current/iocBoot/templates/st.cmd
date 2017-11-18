#!$$IOCTOP/bin/$$IF(ARCH,$$ARCH,linux-x86_64)/ADVisar

< envPaths
epicsEnvSet("IOCNAME", "$$IOCNAME" )
epicsEnvSet("ENGINEER", "$$ENGINEER" )
epicsEnvSet("LOCATION", "$$LOCATION" )
epicsEnvSet("IOCSH_PS1", "$(IOCNAME)> " )
epicsEnvSet("IOC_PV", "$$IOC_PV")
epicsEnvSet("IOCTOP", "$$IOCTOP")
epicsEnvSet("TOP", "$$TOP")
## Add the path to the protocol files
epicsEnvSet("STREAM_PROTOCOL_PATH", "$(IOCTOP)/protocol")

cd( "$(IOCTOP)" )

# Run common startup commands for linux soft IOC's
< /reg/d/iocCommon/All/pre_linux.cmd

# Register all support components
dbLoadDatabase("dbd/ADvisar.dbd")
staubli_registerRecordDeviceDriver(pdbbase)
#------------------------------------------------------------------------------
# Asyn support

## Asyn record support

#dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$$NAME,R=:asyn,PORT=bus$$INDEX,ADDR=0,OMAX=0,IMAX=0")

drvAsynIPPortConfigure ("bus$$INDEX","$$IP:$$PORT TCP", 0, 0, 0)


# Asyn tracing settings

# Load record instances
#dbLoadRecords( "db/iocSoft.db",             "IOC=$(IOC_PV)" )
#dbLoadRecords( "db/save_restoreStatus.db",  "IOC=$(IOC_PV)" )


dbLoadRecords("db/ADVisar.db",       "NAME=$$NAME,bus=bus$$INDEX,IOC_PV=$(IOC_PV)")


# Setup autosave
#set_savefile_path( "$(IOC_DATA)/$(IOCNAME)/autosave" )
#set_requestfile_path( "$(TOP)/autosave" )
#save_restoreSet_status_prefix( "$(IOC_PV):" )
#save_restoreSet_IncompleteSetsOk( 1 )
#save_restoreSet_DatedBackupFiles( 1 )
#set_pass0_restoreFile( "$(IOCNAME).sav" )
#set_pass1_restoreFile( "$(IOCNAME).sav" )

# Initialize the IOC and start processing records
iocInit()

# Start autosave backups
#create_monitor_set( "$(IOCNAME).req", 5, "IOC=$(IOC_PV)" )

# All IOCs should dump some common info after initial startup.
< /reg/d/iocCommon/All/post_linux.cmd

