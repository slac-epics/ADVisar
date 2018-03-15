< envPaths

epicsEnvSet( "IP",        "172.21.46.71"       )
epicsEnvSet( "IP_PORT1",	  "1001"     )
epicsEnvSet( "IP_PORT2",	  "1002"     )
epicsEnvSet( "BASE_NAME", "MEC:VISARj:01"     )
epicsEnvSet( "CALIBATION_FILE",	  "$(TOP)/iocBoot/iocVisarCamera/VISAR1.txt"     )

epicsEnvSet("STREAM_PROTOCOL_PATH", "$(TOP)/db")
epicsEnvSet("AD_PORT", "ADVISAR")

# Specify largest array CA will transport
epicsEnvSet EPICS_CA_MAX_ARRAY_BYTES 20000000

dbLoadDatabase("$(TOP)/dbd/visarCameraApp.dbd")
visarCameraApp_registerRecordDeviceDriver(pdbbase)

# Configure asyn ports
drvAsynIPPortConfigure("PORT_C","$(IP):$(IP_PORT1)", 0, 0, 0)
drvAsynIPPortConfigure("PORT_D","$(IP):$(IP_PORT2)", 0, 0, 0)
dbLoadRecords(	"$(TOP)/db/asynRecord.db",			"P=$(BASE_NAME),R=:PORT_C:AsynIO,PORT=PORT_C,ADDR=0,IMAX=0,OMAX=0" )
dbLoadRecords(	"$(TOP)/db/asynRecord.db",			"P=$(BASE_NAME),R=:PORT_D:AsynIO,PORT=PORT_D,ADDR=0,IMAX=0,OMAX=0" )

epicsThreadSleep(1.0)

dbLoadRecords( "$(TOP)/db/visarCameraIOC.db", "P=$(BASE_NAME),R=:,PORTC=PORT_C,PORTD=PORT_D,PORT=$(AD_PORT),ADDR=0,TIMEOUT=1")

epicsEnvSet("EPICS_DB_INCLUDE_PATH","$(ADCORE)/db")

visarCameraConfig("$(AD_PORT)", "PORT_D" , -1,-1)

# 1344 * 1024 * 2 bytes
epicsEnvSet("NELEMENTS", "5505024")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 3, 0, "$(AD_PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(BASE_NAME):,R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(AD_PORT),NDARRAY_ADDR=0,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

iocInit()

# Wait for enum callbacks to complete
epicsThreadSleep(1.0)


dbpf $(BASE_NAME):image1:EnableCallbacks,1
dbpf $(BASE_NAME):ArrayCallbacks,1
dbpf $(BASE_NAME):Acquire,1
dbpf $(BASE_NAME):AcquireTime,.2
dbpf $(BASE_NAME):ScalingFilePath,$(CALIBATION_FILE)

