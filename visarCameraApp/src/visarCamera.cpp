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

#include "ADDriver.h"

#include <epicsExport.h>

#define DRIVER_VERSION      0
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

#define MAX_OUT_COMMAND_LEN 50
#define MAX_IN_COMMAND_LEN 50

#define SIZE_X 1344
#define SIZE_Y 1024
#define BPP 4

#define NUM_SD_PARAMS  10 //??????????????????



static const char *driverName = "visarCamera";

class visarCamera : public ADDriver
{
public:

    visarCamera(const char *portName, int cameraId, int traceMask, int memoryChannel,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);



  //  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
  //  virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
  //  virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],size_t nElements, size_t *nIn);

  //  void report(FILE *fp, int details);




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


    /* Local methods to this class */

    asynStatus connectCamera();
    asynStatus disconnectCamera();

    asynStatus startCapture();
    asynStatus stopCapture();

    asynStatus grabImage();
    asynStatus waitForFrame();
    asynStatus getRawFrame();


    int exiting_;

    NDArray *pRaw_;


    epicsEventId startEventId_;


};


// Links to C
extern "C" int visarCameraConfig(const char *portName, int cameraId, int traceMask, int memoryChannel,int maxBuffers, size_t maxMemory, int priority, int stackSize){
    new visarCamera( portName, cameraId, traceMask, memoryChannel, maxBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
}

static void c_shutdown(void *arg){
    visarCamera *p = (visarCamera *)arg;
    p->shutdown();
}

static void imageGrabTaskC(void *drvPvt){
    visarCamera *pPvt = (visarCamera *)drvPvt;

    pPvt->imageGrabTask();
}



// Constructor
visarCamera::visarCamera(const char *portName, int cameraId, int traceMask, int memoryChannel, int maxBuffers,size_t maxMemory, int priority, int stackSize)
        : ADDriver(portName, 1, NUM_SD_PARAMS, maxBuffers, maxMemory,0, 0,ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),

          exiting_(0),
          pRaw_(NULL)
{

    static const char *functionName = "visarCamera";
    asynStatus status;


    const char *IPPortName = "localhost";
    IPPortName_ = epicsStrDup(IPPortName);



    /* Create the epicsEvents for signaling to the mythen task when acquisition starts and stops */
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId_) {
        printf("%s:%s epicsEventCreate failure for start event\n",
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

    asynStatus status;

    status = pasynOctetSyncIO->connect(controlPortName_, 0, &pasynUserControl_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for control socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserMeter_->errorMessage);
        return status;
    }

    status = pasynOctetSyncIO->connect(dataPortName_, 0, &pasynUserData_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserMeter_->errorMessage);
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
    Error error;
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
    if (pGuid_) {
        disconnectCamera();
    }
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
             * has started and we can start asking for image buffers...     */
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s waiting for acquire to start\n",
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
        Error error;
        unsigned int nRows, nCols, stride;
        PixelFormat pixelFormat;
        BayerTileFormat bayerFormat;
        NDDataType_t dataType;
        NDColorMode_t colorMode;
        Image *pPGImage;
        ImageMetadata metaData;
        TimeStamp timeStamp;
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
        dataSizePG = pPGRawImage_->GetReceivedDataSize();
        getDoubleParam(FRAME_RATE, PGPropertyValueAbs, &frameRate);
        bandwidth = frameRate * dataSizePG / (1024 * 1024);
        setDoubleParam(PGBandwidth, bandwidth);


        // arranjar a imagem


        // por as coisas no NDArray


        setIntegerParam(NDArraySizeX, nCols);
        setIntegerParam(NDArraySizeY, nRows);
        setIntegerParam(NDArraySize, (int)dataSize);
        setIntegerParam(NDDataType,dataType);
        if (nDims == 3) {
            colorMode = NDColorModeRGB1;
        } else {
            /* If the color mode is currently set to Bayer leave it alone */
            getIntegerParam(NDColorMode, (int *)&colorMode);
            if (colorMode != NDColorModeBayer) colorMode = NDColorModeMono;
        }
        setIntegerParam(NDColorMode, colorMode);


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
        pData = pPGImage->GetData();
        memcpy(pRaw_->pData, pData, dataSize);

        /* Put the frame number into the buffer */
        pRaw_->uniqueId = metaData.embeddedFrameCounter;
        getIntegerParam(PGTimeStampMode, &timeStampMode);
        updateTimeStamp(&pRaw_->epicsTS);

        /* Set the timestamps in the buffer */
        switch (timeStampMode) {
            case TimeStampCamera:
                // Some Point Grey cameras return seconds and microseconds, others cycleSeconds, etc. fields
                if (timeStamp.seconds != 0) {
                    pRaw_->timeStamp = (double)timeStamp.seconds +
                                       (double)timeStamp.microSeconds / 1.e6;
                } else {
                    pRaw_->timeStamp = (double)timeStamp.cycleSeconds +
                                       (double)timeStamp.cycleCount / 8000. +
                                       (double)timeStamp.cycleOffset / 8000. / 3072.;
                }
                break;
            case TimeStampEPICS:
                pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
                break;
            case TimeStampHybrid:
                // For now we just use EPICS time
                pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
                break;
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

    strcpy(frame_,inString_);

    return status;


}





/** Send a string to the detector and reads the response.**/
asynStatus mythen::writeReadControl()
{
    const char *functionName="writeReadMeter";

    size_t read_buffer_len = sizeof(epicsFloat32);

    size_t nread;
    size_t nwrite;
    asynStatus status;
    int eomReason;

    status = pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_),inString_,read_buffer_len , M1K_TIMEOUT, &nwrite, &nread, &eomReason);

    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error!\n",driverName, functionName);
    return status;
}

