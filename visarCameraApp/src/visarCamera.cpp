#include <stdio.h>
#include <string.h>
#include <string>

#include <iostream>


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

#include <asynDriver.h>
#include <asynOctetSyncIO.h>


#include "ADDriver.h"
#include "NDPluginDriver.h"

#include <epicsExport.h>
#include <registryFunction.h>


#define DRIVER_VERSION      0
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

#define MAX_OUT_COMMAND_LEN 50
#define MAX_IN_COMMAND_LEN 50

#define SIZE_X 1344
#define SIZE_Y 1024
#define BPP 2

// Control port timeout
#define VC_C_TIMEOUT 1

// Data port timeout
#define VC_D_TIMEOUT 5

// New Frame timeout
#define VC_F_TIMEOUT 10

#define VisarFrameReadyString "VC_FRAME_READY"
#define VisarStartAcquisitionString "VC_START_ACQ"
#define VisarStopAcquisitionString "VC_STOP_ACQ"
#define VisarGetFrameString "VC_GET_FRAME"
#define VisarNewFrameString "VC_NEW_FRAME"
#define VisarStartAppString "VC_START_APP"
#define VisarStopAppString "VC_STOP_APP"


using namespace std;

typedef enum {
    TimeStampCamera,
    TimeStampEPICS,
    TimeStampHybrid
} PGTimeStamp_t;

static const char *driverName = "visarCamera";

class visarCamera : public ADDriver {
public:

    visarCamera(const char *portName, const char *dataPortName,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    virtual ~visarCamera() {};

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);

    virtual void report(FILE *fp, int details);

    //These should be private but are called from C callback functions, must be public.
    void imageGrabTask();

    void shutdown();

protected:

    // Visar Camera parameters
    int StartAcquisition;
#define FIRST_SD_PARAM StartAcquisition
    int VisarFrameReady;
    int GetFrame;
    int NewFrame;
    int StartApp;
    int StopApp;
    int StopAcquisition;
#define LAST_SD_PARAM StopAcquisition


private:


    const char *dataPortName_;

    asynUser *pasynUserData_;

    // String buffers
    char outString_[MAX_OUT_COMMAND_LEN];
    char inString_[MAX_IN_COMMAND_LEN];
    char rawFrame_[1344 * 1024 * 4];

    epicsEventId startEventId_;
    epicsEventId frameEventId_;

    NDArray *pRaw_;

    asynStatus connectCamera();

    asynStatus disconnectCamera();

    asynStatus startCapture();

    asynStatus stopCapture();

    asynStatus grabImage();

    asynStatus waitForFrame(double *timestamp);

    asynStatus getRawFrame(unsigned int *nRows, unsigned int *nCols, unsigned int *bpp);

    int counter;


};

#define NUM_SD_PARAMS (&LAST_SD_PARAM - &FIRST_SD_PARAM + 1)

// Links to C
extern "C" int visarCameraConfig(const char *portName, const char *dataPortName,
                                 int maxBuffers, size_t maxMemory,
                                 int priority, int stackSize) {
    new visarCamera(portName, dataPortName,
                    maxBuffers, maxMemory, priority, stackSize);
    return (asynSuccess);
}

static void c_shutdown(void *arg) {
    visarCamera *p = (visarCamera *) arg;
    p->shutdown();
}

static void imageGrabTaskC(void *drvPvt) {
    visarCamera *pPvt = (visarCamera *) drvPvt;
    pPvt->imageGrabTask();
}


/** Constructor for visarCamera driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data,
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] controlPortName The asyn network port connection to the Visar Camera control TCP port
  * \param[in] dataPortName The asyn network port connection to the Visar Camera data TCP port
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
visarCamera::visarCamera(const char *portName, const char *dataPortName,
                         int maxBuffers, size_t maxMemory, int priority, int stackSize)

        : ADDriver(portName, 1, NUM_SD_PARAMS, maxBuffers, maxMemory, 0, 0, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1,
                   priority, stackSize),
          pRaw_(NULL) {

    static const char *functionName = "visarCamera";

    int status = asynSuccess;
    dataPortName_ = epicsStrDup(dataPortName);

    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId_) {
        printf("%s:%s epicsEventCreate failure for run event\n", driverName, functionName);
        return;
    }

    frameEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!this->frameEventId_) {
        printf("%s:%s epicsEventCreate failure for run event\n", driverName, functionName);
        return;
    }

    // Set static parameters
    status |= setStringParam(ADManufacturer, "Hamamatsu");
    status |= setIntegerParam(ADMaxSizeX, 1344);
    status |= setIntegerParam(ADMaxSizeY, 1024);
    status |= setIntegerParam(ADMinX, 0);
    status |= setIntegerParam(ADMinY, 0);

    createParam(VisarStartAcquisitionString, asynParamInt32, &StartAcquisition);
    createParam(VisarFrameReadyString, asynParamInt32, &VisarFrameReady);
    createParam(VisarGetFrameString, asynParamInt32, &GetFrame);
    createParam(VisarNewFrameString, asynParamInt32, &NewFrame);
    createParam(VisarStartAppString, asynParamInt32, &StartApp);
    createParam(VisarStopAppString, asynParamInt32, &StopApp);

    createParam(VisarStopAcquisitionString, asynParamInt32, &StopAcquisition);


    status |= connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s:  camera connection failed (%d)\n",
                  driverName, functionName, status);
        return;
    }

    /* Register the shutdown function for epicsAtExit */
    epicsAtExit(c_shutdown, this);

    /* Launch image read task */
    epicsThreadCreate("VisarCameraImageTask",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      imageGrabTaskC, this);

}

/** Connects to camera.
 *  Connects to control and data port.
 *  Starts the Visar camera app*/
asynStatus visarCamera::connectCamera(void) {

    static const char *functionName = "visarCamera::connectCamera";

    int status = asynSuccess;

    status |= pasynOctetSyncIO->connect(dataPortName_, 0, &pasynUserData_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserData_->errorMessage);
        return ((asynStatus) status);
    }

    int startApp;
    getIntegerParam(StartApp, &startApp);
    setIntegerParam(StartApp, !startApp);

    return ((asynStatus) status);

}

/** Disconnects to camera.
 *  Stops the Visar camera app*/
asynStatus visarCamera::disconnectCamera() {

    // static const char *functionName = "visarCamera::disconnectCamera";

    int status = asynSuccess;

    int stopApp;
    getIntegerParam(StopApp, &stopApp);
    setIntegerParam(StopApp, !stopApp);

    return ((asynStatus) status);
}

/** Starts acquisition and activates live monitor. */
asynStatus visarCamera::startCapture() {

    static const char *functionName = "startCapture";

    printf("%s\n", functionName);

    /* Start the camera transmission... */
    setIntegerParam(ADNumImagesCounter, 0);

    int status = asynSuccess;
    int startAcquisition;
    getIntegerParam(StartAcquisition, &startAcquisition);
    status |= setIntegerParam(StartAcquisition, !startAcquisition);


    status |= setIntegerParam(ADAcquire, 1);

    epicsEventSignal(startEventId_);

    counter = 0;

    return ((asynStatus) status);
}

/** Stops acquisition and deactivates live monitor. */
asynStatus visarCamera::stopCapture() {

    static const char *functionName = "stopCapture";
    printf("%s\n", functionName);

    int status = asynSuccess;

    int stopAcquisition;
    getIntegerParam(StopAcquisition, &stopAcquisition);
    status |= setIntegerParam(StopAcquisition, !stopAcquisition);

    status |= setIntegerParam(ADAcquire, 0);

    return ((asynStatus) status);
}

void visarCamera::shutdown(void) {
    stopCapture();
    disconnectCamera();
}

/** Acquisition task. */
void visarCamera::imageGrabTask() {

    static const char *functionName = "imageGrabTask";

    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;

    lock();

    while (1) {

        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {

            //       printf("\n1\n");

            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

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

        //     printf("\n2\n");

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);

        /* We are now waiting for an image  */
        setIntegerParam(ADStatus, ADStatusWaiting);
        /* Call the callbacks to update any changes */
        callParamCallbacks();

        //    printf("\n3\n");

        // Grab a new frame
        status |= grabImage();
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

        /* Put the frame number and time stamp into the buffer */
        pRaw_->uniqueId = imageCounter;
        pRaw_->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
        updateTimeStamp(&pRaw_->epicsTS);


        //    asynPortDriver::updateTimeStamp();

        if (arrayCallbacks) {
            /* Call the NDArray callback */
            /* Must release the lock here, or we can get into a deadlock, because we can
             * block on the plugin lock, and the plugin can be calling us */
            printf("doCallbacksGenericPointer %d\n", imageCounter);
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
            // status |= stopCapture(); ////////////////////////////////////////////////////////////////////////////////////////////// NEED TO FIX THIS
        }
        callParamCallbacks();
    }

}


/** Get a new frame. */
asynStatus visarCamera::grabImage() {

    static const char *functionName = "grabImage";

    //   printf("%s\n",functionName);

    int status = asynSuccess;

    unsigned int nRows, nCols, bpp;
    double timestamp;

    NDDataType_t dataType;
    NDColorMode_t colorMode;
    size_t dims[3];
    size_t dataSize;

    int nDims;
    int acquire;

    /* unlock the driver while we wait for a new image to be ready */
    unlock();


    status |= waitForFrame(&timestamp);


    getIntegerParam(ADAcquire, &acquire);
    if (!acquire) {
        lock();
        return asynError;
    }


    status |= getRawFrame(&nRows, &nCols, &bpp);
    getIntegerParam(ADAcquire, &acquire);
    if (!acquire) {
        lock();
        return asynError;
    }

    lock();

    dataSize = nCols * nRows * bpp;
    if (bpp == 2) {
        dataType = NDUInt16;
        colorMode = NDColorModeMono;
    }

    status |= setIntegerParam(ADSizeX, nCols);
    status |= setIntegerParam(ADSizeY, nRows);

    setIntegerParam(NDArraySizeX, nCols);
    setIntegerParam(NDArraySizeY, nRows);
    setIntegerParam(NDArraySize, (int) dataSize);
    setIntegerParam(NDDataType, dataType);
    setIntegerParam(NDColorMode, colorMode);
    callParamCallbacks();

    nDims = 2;
    dims[0] = nCols;
    dims[1] = nRows;
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
        return (asynError);
    }

    memcpy(pRaw_->pData, rawFrame_, dataSize);

    printf("shipped %d\n", dataSize);

    pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec / 1e9;


    /* Get any attributes that have been defined for this driver */
    getAttributes(pRaw_->pAttributeList);


    /* Change the status to be readout... */
    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();

    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

    return ((asynStatus) status);
}

/** Wait for a new frame notification in the control port. */
asynStatus visarCamera::waitForFrame(double *timestamp) {

    static const char *functionName = "waitForFrame";

    printf("%s\n", functionName);


    int acquire;
    int newFrame;
    int frame;


    while (1) {
        //printf(".");
        getIntegerParam(NewFrame, &newFrame);
        if (newFrame) {
            setIntegerParam(NewFrame, 0);
            callParamCallbacks();
            getIntegerParam(VisarFrameReady, &frame);
            printf("%d\n", frame);

            //      printf("New frame ready \n");
            return asynSuccess;
        }

        getIntegerParam(ADAcquire, &acquire);
        if (!acquire) {
            printf("Acquisition stopped during waiting time \n");
            return asynError;

        }


    }


}

/** Ask for a new frame data and get it from the data port. */
asynStatus visarCamera::getRawFrame(unsigned int *nRows, unsigned int *nCols, unsigned int *bpp) {

    const char *functionName = "getFrame";

    size_t nread;
    int status = asynSuccess;
    int eomReason;
    int getFrame;

    pasynOctetSyncIO->flush(pasynUserData_);

    getIntegerParam(GetFrame, &getFrame);
    setIntegerParam(GetFrame, !getFrame);

    *nRows = 1344;
    *nCols = 1024;
    *bpp = 2;

    size_t read_buffer_len = (*nRows) * (*nCols) * (*bpp);

    status |= pasynOctetSyncIO->read(pasynUserData_, rawFrame_, read_buffer_len, .5, &nread, &eomReason);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: read frame error!\n", driverName, functionName);
    }

    return ((asynStatus) status);
}


// TODO


/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void visarCamera::report(FILE *fp, int details) {
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

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus visarCamera::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;;
    static const char *functionName = "writeInt32";
    int acquire;
    int frame;

    const char *reasonName = "unknownReason";
    getParamName(0, pasynUser->reason, &reasonName);

    // printf(	"\n%s: Write Reason %d %s %d\n", functionName, pasynUser->reason, reasonName,value );


    getIntegerParam(ADAcquire, &acquire);


    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
    if (function == ADAcquire) {
        if (value & !acquire) {
            status |= startCapture();
        } else if (!value & acquire) {
            status |= stopCapture();
        }
    } else if (pasynUser->reason == VisarFrameReady) {
        //  printf("%s","FRAME NOTIFICATION WRITE\n");
        getIntegerParam(function, &frame);
        if (value != frame) {
            setIntegerParam(NewFrame, 1);
            epicsEventSignal(frameEventId_);
            setIntegerParam(function, value);
        }
    } else {
        status |= setIntegerParam(function, value);
    }

    /* Update any changed parameters */
    status |= callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%d\n",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);

    return ((asynStatus) status);
}

/** Called when asyn clients call pasynFlaot64->write().
  * This function performs actions for some parameters, including ADAcquire, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus visarCamera::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;
    static const char *functionName = "writeFloat64";
    int acquire;

    /* Reject any call to the detector if it is running */
    getIntegerParam(ADAcquire, &acquire);
    if ((function != ADAcquire) & acquire) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: detector is busy 64\n", driverName, functionName);
        return asynError;
    }

    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setDoubleParam(function, value);

    /* Update any changed parameters */
    status |= callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%d\n",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);

    return ((asynStatus) status);
}


asynStatus visarCamera::readInt32(asynUser *pasynUser, epicsInt32 *pValueRet) {
    // static const char	*	functionName	= "readInt32";
    //const char *reasonName = "unknownReason";
    // getParamName( 0, pasynUser->reason, &reasonName );

    //   printf(	"%s: Reason %d %s\n", functionName, pasynUser->reason, reasonName );


    // Call base class
    asynStatus status = ADDriver::readInt32(pasynUser, pValueRet);
    return ((asynStatus) status);
}


/** Code for iocsh registration */

static const iocshArg visarCameraConfigArg0 = {"Port name", iocshArgString};
static const iocshArg visarCameraConfigArg1 = {"Data asyn port name", iocshArgString};
static const iocshArg visarCameraConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg visarCameraConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg visarCameraConfigArg4 = {"priority", iocshArgInt};
static const iocshArg visarCameraConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg *const visarCameraConfigArgs[] = {&visarCameraConfigArg0,
                                                        &visarCameraConfigArg1,
                                                        &visarCameraConfigArg2,
                                                        &visarCameraConfigArg3,
                                                        &visarCameraConfigArg4,
                                                        &visarCameraConfigArg5,
                                                        };


static const iocshFuncDef configvisarcamera = {"visarCameraConfig", 6, visarCameraConfigArgs};

static void configvisarcameraCallFunc(const iocshArgBuf *args) {
    visarCameraConfig(args[0].sval, args[1].sval,
                      args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void visarCameraRegister(void) {

    iocshRegister(&configvisarcamera, configvisarcameraCallFunc);
}

extern "C" {
epicsExportRegistrar(visarCameraRegister);
}







//static long my_asub_routine(aSubRecord *prec) {
//    long i, *a;
//    double sum=0;

//    a = (long *)prec->a;
//    for (i=0; i<prec->noa; i++) {
//        sum += a[i];
//    }

//    return 0; /* process output links */
//}
//epicsRegisterFunction(my_asub_routine);