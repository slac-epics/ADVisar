#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <tchar.h>
  #include <windows.h>
#else
#include <unistd.h>
#endif

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEndian.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"
#include "NDPluginDriver.h"

#include <epicsExport.h>

#define DRIVER_VERSION      0
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

#define MAX_OUT_COMMAND_LEN 50
#define MAX_IN_COMMAND_LEN 50

#define SIZE_X 1344
#define SIZE_Y 1024
#define BPP 4

#define NUM_SD_PARAMS  100 //??????????????????

#define M1K_TIMEOUT 10 //?????????/



static const char *driverName = "visarCamera";

class visarCamera : public ADDriver
{
public:

    visarCamera(const char *portName, const char *controlPortName, const char *dataPortName,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    virtual ~visarCamera(){};



    virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
//    virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value){};
//    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],size_t nElements, size_t *nIn);

    virtual  void report(FILE *fp, int details);
//    virtual void setShutter(int open);




    /**< These should be private but are called from C callback functions, must be public. */
    void imageGrabTask();
    void shutdown();

protected:

    // Properties


private:

    const char *controlPortName_ ;
    const char *dataPortName_;

    asynUser *pasynUserControl_;
    asynUser *pasynUserData_;

    char outString_[MAX_OUT_COMMAND_LEN];
    char inString_[MAX_IN_COMMAND_LEN];
    char rawFrame_[SIZE_X*SIZE_Y*BPP];

    epicsEventId startEventId_;

    int acquiring_;
    int exiting_;

    NDArray *pRaw_;



    /* Local methods to this class */

    asynStatus connectCamera();
    asynStatus disconnectCamera();

    asynStatus startCapture();
    asynStatus stopCapture();

    asynStatus grabImage();
    asynStatus waitForFrame();
    asynStatus getRawFrame();


};


// Links to C
extern "C" int visarCameraConfig(const char *portName, const char *controlPortName, const char *dataPortName,
                            int maxBuffers, size_t maxMemory,
                            int priority, int stackSize)

{
    new visarCamera(portName, controlPortName,dataPortName,
               maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

static void c_shutdown(void *arg){
    visarCamera *p = (visarCamera *)arg;
    p->shutdown();
}

static void imageGrabTaskC(void *drvPvt){
    visarCamera *pPvt = (visarCamera *)drvPvt;

    pPvt->imageGrabTask();
}



asynStatus visarCamera::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    return (asynSuccess);
}

/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void visarCamera::report(FILE *fp, int details)
{
    fprintf(fp, "visarCamera %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}



// Constructor
visarCamera::visarCamera(const char *portName, const char *controlPortName, const char *dataPortName,
                         int maxBuffers,size_t maxMemory, int priority, int stackSize)

        : ADDriver(portName, 1, NUM_SD_PARAMS, maxBuffers, maxMemory,0, 0,ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),

          exiting_(0),
          pRaw_(NULL)
{

    static const char *functionName = "visarCamera";

    asynStatus status = asynSuccess;



    controlPortName_ = epicsStrDup(controlPortName);
    dataPortName_ = epicsStrDup(dataPortName);



    /* Create the epicsEvents for signaling to the mythen task when acquisition starts and stops */
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId_) {
        printf("%s:%s epicsEventCreate failure for run event\n",
               driverName, functionName);
        return;
    }


    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s:  camera connection failed (%d)\n",
                  driverName, functionName, status);
        return;
    }

    /* Register the shutdown function for epicsAtExit */
    epicsAtExit(c_shutdown, this);


    /* launch image read task */
    epicsThreadCreate("VisarCameraImageTask",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      imageGrabTaskC, this);

}

asynStatus visarCamera::connectCamera(void) {

    static const char *functionName = "visarCamera::connectCamera";

    asynStatus status;

    status = pasynOctetSyncIO->connect(controlPortName_, 0, &pasynUserControl_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for control socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserControl_->errorMessage);
        return status;
    }

    status = pasynOctetSyncIO->connect(dataPortName_, 0, &pasynUserData_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserData_->errorMessage);
        return status;
    }

    return asynSuccess;

}

asynStatus visarCamera::disconnectCamera() {

    return asynSuccess;
}

asynStatus visarCamera::startCapture() {

    static const char *functionName = "startCapture";

    /* Start the camera transmission... */
    setIntegerParam(ADNumImagesCounter, 0);


    size_t nread;
    size_t nwrite;
    asynStatus status;
    int eomReason;

    strcpy(outString_, "AcqStart(Live)");
    status = pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_),inString_,strlen(inString_) , M1K_TIMEOUT, &nwrite, &nread, &eomReason);
    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: error!\n",driverName, functionName);

    strcpy(outString_, "AcqLiveMonitor(NotifyTimeStamp)");
    status = pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_),inString_,strlen(inString_) , M1K_TIMEOUT, &nwrite, &nread, &eomReason);
    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: error!\n",driverName, functionName);


    epicsEventSignal(startEventId_);
    return asynSuccess;
}

asynStatus visarCamera::stopCapture(){

    static const char *functionName = "stopCapture";

    size_t nread;
    size_t nwrite;
    asynStatus status;
    int eomReason;

    strcpy(outString_, "AcqStop()");
    status = pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_),inString_,strlen(inString_) , M1K_TIMEOUT, &nwrite, &nread, &eomReason);
    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: error!\n",driverName, functionName);

    return asynSuccess;
}


void visarCamera::shutdown(void){
    exiting_ = 1;
  //  if (pGuid_) {
        disconnectCamera();
  //  }
}


void visarCamera::imageGrabTask() {

    asynStatus status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;
    static const char *functionName = "imageGrabTask";

    lock();

    while (1) {

        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();                       // what is this?

            /* Wait for a signal that tells this thread that the transmission
             * has started and we can run asking for image buffers...     */
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s waiting for acquire to run\n",
                      driverName, functionName);
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            unlock();

            epicsEventWait(startEventId_);
            lock();

            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s started!\n",
                      driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
        }

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);

        /* We are now waiting for an image  */
        setIntegerParam(ADStatus, ADStatusWaiting);
        /* Call the callbacks to update any changes */
        callParamCallbacks();


        status = grabImage();
        if (status == asynError) {
            /* remember to release the NDArray back to the pool now
             * that we are not using it (we didn't get an image...) */
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;
            continue;
        }


        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        if (arrayCallbacks) {
            /* Call the NDArray callback */
            /* Must release the lock here, or we can get into a deadlock, because we can
             * block on the plugin lock, and the plugin can be calling us */
            unlock();
            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
            lock();
        }

        /* Release the NDArray buffer now that we are done with it.
        * After the callback just above we don't need it anymore */
        pRaw_->release();
        pRaw_ = NULL;

        /* See if acquisition is done if we are in single or multiple mode */
        if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
            status = stopCapture();
        }
        callParamCallbacks();

    }

}

asynStatus visarCamera::grabImage() {
        asynStatus status = asynSuccess;

        unsigned int nRows, nCols, stride;

        NDDataType_t dataType;
        NDColorMode_t colorMode;
        int convertPixelFormat;
        int numColors;
        size_t dims[3];
        int pixelSize;
        size_t dataSize, dataSizePG;
        double bandwidth;
        double frameRate;
        void *pData;
        int nDims;
        int timeStampMode;
        static const char *functionName = "grabImage";



        /* unlock the driver while we wait for a new image to be ready */
        unlock();
        status = waitForFrame();

        //get the image
        status = getRawFrame();

        lock();

        // Calculate bandwidth
       // dataSizePG = pPGRawImage_->GetReceivedDataSize();
    //    getDoubleParam(FRAME_RATE, PGPropertyValueAbs, &frameRate);
  //      bandwidth = frameRate * dataSizePG / (1024 * 1024);
 //       setDoubleParam(PGBandwidth, bandwidth);


        // arranjar a imagem


        // por as coisas no NDArray


   //     setIntegerParam(NDArraySizeX, nCols);
   //     setIntegerParam(NDArraySizeY, nRows);
   //     setIntegerParam(NDArraySize, (int)dataSize);
   //     setIntegerParam(NDDataType,dataType);
   //     if (nDims == 3) {
   //         colorMode = NDColorModeRGB1;
   //     } else {
   //         /* If the color mode is currently set to Bayer leave it alone */
   //         getIntegerParam(NDColorMode, (int *)&colorMode);
   //         if (colorMode != NDColorModeBayer) colorMode = NDColorModeMono;
   //     }
   //     setIntegerParam(NDColorMode, colorMode);


        pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);

        if (!pRaw_) {
            /* If we didn't get a valid buffer from the NDArrayPool we must abort
             * the acquisition as we have nowhere to dump the data...       */
            setIntegerParam(ADStatus, ADStatusAborting);
            callParamCallbacks();
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
                      driverName, functionName, portName);
            setIntegerParam(ADAcquire, 0);
            return(asynError);
        }

        // copia a imagem para o buffer do NDArrayPool
    //    pData = pPGImage->GetData();
    //    memcpy(pRaw_->pData, pData, dataSize);

        /* Put the frame number into the buffer */
  //      pRaw_->uniqueId = metaData.embeddedFrameCounter;
   //     getIntegerParam(PGTimeStampMode, &timeStampMode);
 //       updateTimeStamp(&pRaw_->epicsTS);

        /* Set the timestamps in the buffer */
        switch (timeStampMode) {
       //     case TimeStampCamera:
                // Some Point Grey cameras return seconds and microseconds, others cycleSeconds, etc. fields
        //        if (timeStamp.seconds != 0) {
        //            pRaw_->timeStamp = (double)timeStamp.seconds +
       //                                (double)timeStamp.microSeconds / 1.e6;
        //        } else {
        //            pRaw_->timeStamp = (double)timeStamp.cycleSeconds +
         //                              (double)timeStamp.cycleCount / 8000. +
         //                              (double)timeStamp.cycleOffset / 8000. / 3072.;
        //        }
//                break;
       //     case TimeStampEPICS:
      //          pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
     //           break;
    //        case TimeStampHybrid:
                // For now we just use EPICS time
    //            pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
//                break;
        }


        /* Get any attributes that have been defined for this driver */
        getAttributes(pRaw_->pAttributeList);

        /* Change the status to be readout... */
        setIntegerParam(ADStatus, ADStatusReadout);
        callParamCallbacks();

        pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

        return status;



}

asynStatus visarCamera::waitForFrame(void) {

    asynStatus status;


    // Wait for           4,LiveMonitor,notifytimestamp,timestamp

    return status;


}

asynStatus visarCamera::getRawFrame(void) {

    const char *functionName="getFrame";

    int bpp = 4;

    strcpy(outString_, "ImgDataGet(Current,Data)");
    size_t read_buffer_len = 1344*1024*bpp;
    size_t nread;
    size_t nwrite;

    asynStatus status;
    int eomReason;

    status = pasynOctetSyncIO->writeRead(pasynUserData_, outString_, strlen(outString_),inString_,read_buffer_len , M1K_TIMEOUT, &nwrite, &nread, &eomReason);

    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: error!\n",driverName, functionName);

    strcpy(rawFrame_,inString_);

    return status;


}


/* Code for iocsh registration */

static const iocshArg visarCameraConfigArg0 = {"Port name", iocshArgString};
static const iocshArg visarCameraConfigArg1 = {"Control asyn port name", iocshArgString};
static const iocshArg visarCameraConfigArg2 = {"Data asyn port name", iocshArgString};
static const iocshArg visarCameraConfigArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg visarCameraConfigArg4 = {"maxMemory", iocshArgInt};
static const iocshArg visarCameraConfigArg5 = {"priority", iocshArgInt};
static const iocshArg visarCameraConfigArg6 = {"stackSize", iocshArgInt};
static const iocshArg * const visarCameraConfigArgs[] =  {&visarCameraConfigArg0,
                                                     &visarCameraConfigArg1,
                                                     &visarCameraConfigArg2,
                                                     &visarCameraConfigArg3,
                                                     &visarCameraConfigArg4,
                                                     &visarCameraConfigArg5,
                                                     &visarCameraConfigArg6,};


static const iocshFuncDef configvisarcamera = {"visarCameraConfig", 7, visarCameraConfigArgs};
static void configvisarcameraCallFunc(const iocshArgBuf *args)
{
    visarCameraConfig(args[0].sval, args[1].sval, args[2].sval,
                 args[3].ival, args[4].ival,  args[5].ival ,args[6].ival);
}

static void visarCameraRegister(void)
{

    iocshRegister(&configvisarcamera, configvisarcameraCallFunc);
}

extern "C" {
epicsExportRegistrar(visarCameraRegister);
}






///** Send a string to the detector and reads the response.**/
//asynStatus mythen::writeReadControl()
//{
//    const char *functionName="writeReadMeter";
//
//    size_t read_buffer_len = sizeof(epicsFloat32);
//
//    size_t nread;
//    size_t nwrite;
//    asynStatus status;
//    int eomReason;
//
//    status = pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_),inString_,read_buffer_len , M1K_TIMEOUT, &nwrite, &nread, &eomReason);
//
//    if(status != asynSuccess)
//        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
//                  "%s:%s: error!\n",driverName, functionName);
//    return status;
//}

