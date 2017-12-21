< envPaths




epicsEnvSet( "IP",        "172.21.46.71"       )
epicsEnvSet( "IP_PORT1",	  "1001"     )
epicsEnvSet( "IP_PORT2",	  "1002"     )
epicsEnvSet( "BASE_NAME", "MEC:VISAR:02"     )
epicsEnvSet("STREAM_PROTOCOL_PATH", "$(TOP)/db")

epicsEnvSet("PREFIX","$(BASE_NAME):")
epicsEnvSet("PORT_C",   "PORT_C")
epicsEnvSet("PORT_D",   "PORT_D")


# Specify largest array CA will transport
# Note for N sscanRecord data points, need (N+1)*8 bytes, else MEDM
# plot doesn't display
epicsEnvSet EPICS_CA_MAX_ARRAY_BYTES 64008


dbLoadDatabase("$(TOP)/dbd/visarCameraApp.dbd")
visarCameraApp_registerRecordDeviceDriver(pdbbase)


drvAsynIPPortConfigure("PORT_C","$(IP):$(IP_PORT1)", 0, 0, 0)
drvAsynIPPortConfigure("PORT_D","$(IP):$(IP_PORT2)", 0, 0, 0)

#asynOctetSetInputEos("PORT_C",0,"\r\n")
#asynOctetSetOutputEos("PORT_C",0,"\r")
#asynOctetSetInputEos("PORT_D",0,"\r\n")
#asynOctetSetOutputEos("PORT_D",0,"\r")

#asynSetTraceFile( "PORT_C", 0, "$(IOC_DATA)/$(IOC)/iocInfo/asynControl.log" )
#asynSetTraceFile( "PORT_D", 0, "$(IOC_DATA)/$(IOC)/iocInfo/asynData.log" )


#define ASYN_TRACE_ERROR     0x0001
#define ASYN_TRACEIO_DEVICE  0x0002
#define ASYN_TRACEIO_FILTER  0x0004
#define ASYN_TRACEIO_DRIVER  0x0008
#define ASYN_TRACE_FLOW      0x0010

#asynSetTraceMask( "PORT_C", 0, 0xff) # log everything
#asynSetTraceIOMask( "PORT_C", 0, 2)
asynSetTraceMask( "PORT_D", 0, 0xff) # log everything
asynSetTraceIOMask( "PORT_D", 0, 2)





epicsEnvSet("PORT", "VC1")
epicsEnvSet("QSIZE",  "2000")
epicsEnvSet("XSIZE",  "1344")
epicsEnvSet("YSIZE",  "1024")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("EPICS_DB_INCLUDE_PATH","$(ADCORE)/db")
epicsEnvSet("NELEMENTS", "5505024")


visarCameraConfig("$(PORT)", "$(PORT_C)", "$(PORT_D)" , -1,-1)
asynSetTraceIOMask($(PORT), 0, 2)


dbLoadRecords( "$(TOP)/db/visarCameraIOC.db", "P=$(BASE_NAME),R=:,PORTC=$(PORT_C),PORTD=$(PORT_D),PORT=$(PORT),ADDR=0,TIMEOUT=1")



# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=:image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NCHANS)")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADVISARCAMERA)/visarCameraApp/Db")

## Load asynRecord records on Mythen communication
#dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX),R=asyn_1,PORT=IP_M1K,ADDR=0,OMAX=256,IMAX=256")


iocInit()

# save things every thirty seconds
#create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")


# Wait for enum callbacks to complete
epicsThreadSleep(1.0)
