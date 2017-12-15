< envPaths


ENGINEER=joaoprod
NAME=MEC:VISAR:01
LOCATION=MEC:VISAR:IOC:01
IP=172.21.46.71
IP_PORT1=1001
IP_PORT2=1002
IOCPVROOT=MEC:VISAR:IOC:01



epicsEnvSet( "IP",        "172.21.46.71"       )
epicsEnvSet( "IP_PORT1",	  "1001"     )
epicsEnvSet( "IP_PORT2",	  "1002"     )
epicsEnvSet( "BASE_NAME", "MEC:VISAR:01"     )
epicsEnvSet( "IOCTOP",    "$$IOCTOP"   )
epicsEnvSet( "TOP",       "$$TOP"      )
epicsEnvSet("STREAM_PROTOCOL_PATH", "$(IOCTOP)/protocol")




# Specify largest array CA will transport
# Note for N sscanRecord data points, need (N+1)*8 bytes, else MEDM
# plot doesn't display
epicsEnvSet EPICS_CA_MAX_ARRAY_BYTES 64008

dbLoadDatabase("$(TOP)/dbd/visarCameraApp.dbd")

visarCameraApp_registerRecordDeviceDriver(pdbbase)


drvAsynIPPortConfigure("PORT_C","$(IP):$(IP_PORT1)", 0, 0, 0)
drvAsynIPPortConfigure("PORT_D","$(IP):$(IP_PORT2)", 0, 0, 0)

asynOctetSetInputEos("PORT_C",0,"\r\n")
asynOctetSetOutputEos("PORT_C",0,"\r")
asynOctetSetInputEos("PORT_D",0,"\r\n")
asynOctetSetOutputEos("PORT_D",0,"\r")

asynSetTraceFile( "PORT_C", 0, "$(IOC_DATA)/$(IOC)/iocInfo/asynControl.log" )
asynSetTraceFile( "PORT_D", 0, "$(IOC_DATA)/$(IOC)/iocInfo/asynData.log" )


#define ASYN_TRACE_ERROR     0x0001
#define ASYN_TRACEIO_DEVICE  0x0002
#define ASYN_TRACEIO_FILTER  0x0004
#define ASYN_TRACEIO_DRIVER  0x0008
#define ASYN_TRACE_FLOW      0x0010
asynSetTraceMask( "PORT_C", 0, 0xff) # log everything
asynSetTraceIOMask( "PORT_C", 0, 2)
asynSetTraceMask( "PORT_D", 0, 0xff) # log everything
asynSetTraceIOMask( "PORT_D", 0, 2)






epicsEnvSet("PREFIX","visarCamera1:")
epicsEnvSet("PORT_C",   "PORT_C")
epicsEnvSet("PORT_D",   "PORT_D")

epicsEnvSet("QSIZE",  "2000")
epicsEnvSet("XSIZE",  "1344")
epicsEnvSet("YSIZE",  "1024")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("EPICS_DB_INCLUDE_PATH","$(ADCORE)/db")

# mythenConfig (
#               portName,       # The name of the asyn port driver to be created.
#               IPPortName,     # The network port connection to the Mythen
#               maxBuffers,     # The maximum number of NDArray buffers that the NDArrayPool for this driver is
#                                 allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
#               maxMemory)      # The maximum amount of memory that the NDArrayPool for this driver is
#                                 allowed to allocate. Set this to -1 to allow an unlimited amount of memory.

visarCameraConfig("$(PORT)", "$(PORT_C)", "$(PORT_D)" , -1,-1)


dbLoadRecords( "$(ADVISARCAMERA)/mythenApp/Db/visarCamera.template"", "P=$(BASE_NAME),R=:,PORT_C=$(PORT_C),PORT_D=$(PORT_D)")



# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(NCHANS)")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd

# Load asynRecord records on Mythen communication
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX),R=asyn_1,PORT=IP_M1K,ADDR=0,OMAX=256,IMAX=256")

set_requestfile_path("$(TOP)/visarCameraApp/Db")

iocInit()

# save things every thirty seconds
#create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")


