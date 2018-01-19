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
#define BPP 2

// Control port timeout
#define VC_C_TIMEOUT 1

// Data port timeout
#define VC_D_TIMEOUT 5

// New Frame timeout
#define VC_F_TIMEOUT 10

#define SDFrameRateString "VC_FRAMERATE"
#define SDAppStatusString "VC_APPSTATUS"

using namespace std;

typedef enum {
    TimeStampCamera,
    TimeStampEPICS,
    TimeStampHybrid
} PGTimeStamp_t;

static const char *driverName = "visarCamera";

class visarCamera : public ADDriver {
public:

    visarCamera(const char *portName, const char *controlPortName, const char *dataPortName,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    virtual ~visarCamera() {};

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    // virtual asynStatus readInt32( asynUser *pasynUser, epicsInt32 *value);

    virtual void report(FILE *fp, int details);

    //These should be private but are called from C callback functions, must be public.
    void imageGrabTask();
    void shutdown();

protected:

    // Visar Camera parameters
    int SDFrameRate;
    #define FIRST_SD_PARAM SDFrameRate
    int SDAppStatus;
    #define LAST_SD_PARAM SDAppStatus


private:

    const char *controlPortName_;
    const char *dataPortName_;

    asynUser *pasynUserControl_;
    asynUser *pasynUserData_;

    // String buffers
    char outString_[MAX_OUT_COMMAND_LEN];
    char inString_[MAX_IN_COMMAND_LEN];
    char rawFrame_[1344 * 1024 * 4];

    epicsEventId startEventId_;

    NDArray *pRaw_;

    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus startCapture();
    asynStatus stopCapture();
    asynStatus grabImage();
    asynStatus waitForFrame(double *timestamp);
    asynStatus getRawFrame(unsigned int *nRows, unsigned int *nCols, unsigned int *bpp);
};
#define NUM_SD_PARAMS (&LAST_SD_PARAM - &FIRST_SD_PARAM + 1)

// Links to C
extern "C" int visarCameraConfig(const char *portName, const char *controlPortName, const char *dataPortName,
                                 int maxBuffers, size_t maxMemory,
                                 int priority, int stackSize) {
    new visarCamera(portName, controlPortName, dataPortName,
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

//
/** String split function  */
char *zstring_strtok(char *str, const char *delim) {
    static char *static_str = 0;      /* var to store last address */
    int index = 0, strlength = 0;       /* integers for indexes */
    int found = 0;                  /* check if delim is found */

    /* delimiter cannot be NULL
    * if no more char left, return NULL as well
    */
    if (delim == 0 || (str == 0 && static_str == 0))
        return 0;

    if (str == 0)
        str = static_str;

    /* get length of string */
    while (str[strlength])
        strlength++;

    /* find the first occurance of delim */
    for (index = 0; index < strlength; index++)
        if (str[index] == delim[0]) {
            found = 1;
            break;
        }

    /* if delim is not contained in str, return str */
    if (!found) {
        static_str = 0;
        return str;
    }

    /* check for consecutive delimiters
    *if first char is delim, return delim
    */
    if (str[0] == delim[0]) {
        static_str = (str + 1);
        return (char *) delim;
    }

    /* terminate the string
    * this assignmetn requires char[], so str has to
    * be char[] rather than *char
    */
    str[index] = '\0';

    /* save the rest of the string */
    if ((str + index + 1) != 0)
        static_str = (str + index + 1);
    else
        static_str = 0;

    return str;
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
visarCamera::visarCamera(const char *portName, const char *controlPortName, const char *dataPortName,
                         int maxBuffers, size_t maxMemory, int priority, int stackSize)

        : ADDriver(portName, 1, NUM_SD_PARAMS, maxBuffers, maxMemory, 0, 0, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1,
                   priority, stackSize),
          pRaw_(NULL) {

    static const char *functionName = "visarCamera";

    int status = asynSuccess;
    controlPortName_ = epicsStrDup(controlPortName);
    dataPortName_ = epicsStrDup(dataPortName);

    // Create the epicsEvents for signaling to the mythen task when acquisition starts and stops
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId_) {
        printf("%s:%s epicsEventCreate failure for run event\n",driverName, functionName);
        return;
    }

    // Set static parameters
    status |= setStringParam(ADManufacturer, "Hamamatsu");
    status |= setIntegerParam(ADMaxSizeX, 1344);
    status |= setIntegerParam(ADMaxSizeY, 1024);
    status |= setIntegerParam(ADMinX, 0);
    status |= setIntegerParam(ADMinY, 0);

    createParam(SDFrameRateString, asynParamFloat64, &SDFrameRate);
    createParam(SDAppStatusString, asynParamInt32, &SDAppStatus);

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

    status |= pasynOctetSyncIO->connect(controlPortName_, 0, &pasynUserControl_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for control socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserControl_->errorMessage);
        return((asynStatus)status);
    }

    status |= pasynOctetSyncIO->connect(dataPortName_, 0, &pasynUserData_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserData_->errorMessage);
        return((asynStatus)status);
    }

    char ch = '\r';
    status |= pasynOctetSyncIO->setInputEos(pasynUserControl_, &ch, 1);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->setInputEos for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserControl_->errorMessage);
        return((asynStatus)status);
    }

    ch = '\r';
    status |= pasynOctetSyncIO->setOutputEos(pasynUserControl_, &ch, 1);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->setOutputEos for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserControl_->errorMessage);
        return((asynStatus)status);
    }


    // Start the aplication
    size_t nread;
    size_t nwrite;
    int eomReason;


    strcpy(outString_, "AppStart()");
    status |= pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_),inString_,MAX_IN_COMMAND_LEN , VC_C_TIMEOUT, &nwrite, &nread, &eomReason);
    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: error!\n",driverName, functionName);
    return((asynStatus)status);
}

/** Disconnects to camera.
 *  Stops the Visar camera app*/
asynStatus visarCamera::disconnectCamera() {

    static const char *functionName = "visarCamera::disconnectCamera";

    int status = asynSuccess;

    // Stop the aplication
    size_t nread;
    size_t nwrite;
    int eomReason;

    strcpy(outString_, "AppEnd()");
    status |= pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_),inString_,MAX_IN_COMMAND_LEN , VC_C_TIMEOUT, &nwrite, &nread, &eomReason);
    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: error!\n",driverName, functionName);
    return((asynStatus)status);
}

/** Starts acquisition and activates live monitor. */
asynStatus visarCamera::startCapture() {

    static const char *functionName = "startCapture";

    /* Start the camera transmission... */
    setIntegerParam(ADNumImagesCounter, 0);

    size_t nread;
    size_t nwrite;
    int status = asynSuccess;
    int eomReason;

    strcpy(outString_, "AcqStart(Live)");
    status |= pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_), inString_,
                                         MAX_IN_COMMAND_LEN, VC_C_TIMEOUT, &nwrite, &nread, &eomReason);
    if (status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: error!\n", driverName, functionName);

    strcpy(outString_, "AcqLiveMonitor(NotifyTimeStamp)");
    status |= pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_), inString_,
                                         MAX_IN_COMMAND_LEN, VC_C_TIMEOUT, &nwrite, &nread, &eomReason);
    if (status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: error!\n", driverName, functionName);

    epicsEventSignal(startEventId_);

    status |= setIntegerParam(ADAcquire, 1);

    return((asynStatus)status);
}

/** Stops acquisition and deactivates live monitor. */
asynStatus visarCamera::stopCapture() {

    static const char *functionName = "stopCapture";

    size_t nread;
    size_t nwrite;
    int status = asynSuccess;
    int eomReason;

    strcpy(outString_, "AcqStop()");
    status |= pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_), inString_,
                                         MAX_IN_COMMAND_LEN, VC_C_TIMEOUT, &nwrite, &nread, &eomReason);
    if (status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: error!\n", driverName, functionName);

    strcpy(outString_, "AcqLiveMonitor(Off)");
    status |= pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_), inString_,
                                         MAX_IN_COMMAND_LEN, VC_C_TIMEOUT, &nwrite, &nread, &eomReason);
    if (status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: error!\n", driverName, functionName);

    status |= setIntegerParam(ADAcquire, 0);

    return((asynStatus)status);
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

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);

        /* We are now waiting for an image  */
        setIntegerParam(ADStatus, ADStatusWaiting);
        /* Call the callbacks to update any changes */
        callParamCallbacks();

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
           // status |= stopCapture(); ////////////////////////////////////////////////////////////////////////////////////////////// NEED TO FIX THIS
        }
        callParamCallbacks();
    }

}

/** Get a new frame. */
asynStatus visarCamera::grabImage() {

    static const char *functionName = "grabImage";

    int status = asynSuccess;

    unsigned int nRows, nCols, bpp;
    double timestamp;

    NDDataType_t dataType;
    NDColorMode_t colorMode;
    size_t dims[3];
    size_t dataSize;
    double bandwidth;
    double frameRate;
    int nDims;
    int timeStampMode = TimeStampCamera;   //////////////////////
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

    // Calculate bandwidth
    dataSize = nCols * nRows * bpp;
    getDoubleParam(SDFrameRate, &frameRate);
    bandwidth = frameRate * dataSize / (1024 * 1024);
    //       setDoubleParam(PGBandwidth, bandwidth);

    if (bpp == 2) {
        dataType = NDUInt16;
        colorMode = NDColorModeMono;

    } else if (bpp == 4) {
        dataType = NDUInt32;
        colorMode = NDColorModeMono;
    }

    status |= setIntegerParam(ADSizeX, nCols);
    status |= setIntegerParam(ADSizeY, nRows);

    setIntegerParam(NDArraySizeX, nCols);
    setIntegerParam(NDArraySizeY, nRows);
    setIntegerParam(NDArraySize, (int) dataSize);
    setIntegerParam(NDDataType, dataType);
    setIntegerParam(NDColorMode, colorMode);

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

    /* Put the frame number into the buffer */
    //      pRaw_->uniqueId = metaData.embeddedFrameCounter;
    //     getIntegerParam(PGTimeStampMode, &timeStampMode);
    //       updateTimeStamp(&pRaw_->epicsTS);

    // getIntegerParam(PGTimeStampMode, &timeStampMode);
    /* Set the timestamps in the buffer */
    switch (timeStampMode) {
        case TimeStampCamera:
            pRaw_->timeStamp = timestamp;
            break;
        case TimeStampEPICS:
            pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec / 1e9;
            break;
        case TimeStampHybrid:
            // For now we just use EPICS time
            pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec / 1e9;
            break;
    }

    /* Get any attributes that have been defined for this driver */
    getAttributes(pRaw_->pAttributeList);

    /* Change the status to be readout... */
    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();

    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

    return((asynStatus)status);
}

/** Wait for a new frame notification in the control port. */
asynStatus visarCamera::waitForFrame(double *timestamp) {

    static const char *functionName = "waitForFrame";

    int status = asynSuccess;
    size_t nread;
    int eomReason;
    int acquire;
    char timestamp_s[30];

    while (1) {
        pasynOctetSyncIO->flush(pasynUserControl_);
        status |= pasynOctetSyncIO->read(pasynUserControl_, inString_, MAX_IN_COMMAND_LEN, VC_F_TIMEOUT, &nread,&eomReason);
        if (status != asynSuccess) {
            //TODO do not print if the acquisition time has not passed yet
            // TEST
   //         double acquireTime;
   //         getDoubleParam(ADAcquireTime, &acquireTime);
    //        printf("> aq time : %f\n", acquireTime);

            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: error!\n", driverName, functionName);
        }
//        printf(inString_);
//        printf("\n");

        if (strncmp(inString_, "4,Frame rate", 12) == 0) {
            zstring_strtok(inString_, " ");
            zstring_strtok(NULL, " ");
            setDoubleParam(SDFrameRate, atof(zstring_strtok(NULL, " ")));
        } else if (strncmp(inString_, "4,Livemonitor,notifytimestamp,", 30) == 0) {
            printf(">frame\n");

            strncpy(timestamp_s, inString_ + 30, nread - 30);
            *timestamp = atof(timestamp_s);
            break;
        }

        getIntegerParam(ADAcquire, &acquire);
        if (!acquire) {
            break;
        }
    }
    return((asynStatus)status);
}

/** Ask for a new frame data and get it from the data port. */
asynStatus visarCamera::getRawFrame(unsigned int *nRows, unsigned int *nCols, unsigned int *bpp) {

    const char *functionName = "getFrame";

    size_t nread;
    size_t nwrite;
    int status = asynSuccess;
    int eomReason;

    strcpy(outString_, "ImgDataGet(Current,Data)");
    status |= pasynOctetSyncIO->writeRead(pasynUserControl_, outString_, strlen(outString_), inString_,
                                         MAX_IN_COMMAND_LEN, VC_C_TIMEOUT, &nwrite, &nread, &eomReason);
    if (status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: error!\n", driverName, functionName);

 //   printf(inString_);
 //   printf("\n");

    // Get nRows, nCols and bpp from the visar response
    zstring_strtok(inString_, ",");
    zstring_strtok(NULL, ",");
    *nRows = (unsigned int) atoi(zstring_strtok(NULL, ","));
    *nCols = (unsigned int) atoi(zstring_strtok(NULL, ","));
    *bpp = (unsigned int) atoi(zstring_strtok(NULL, ","));

    size_t read_buffer_len = (*nRows) * (*nCols) * (*bpp);

    pasynOctetSyncIO->flush(pasynUserData_);
    status |= pasynOctetSyncIO->read(pasynUserData_, rawFrame_,read_buffer_len, VC_D_TIMEOUT, &nread, &eomReason);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: read frame error!\n", driverName, functionName);
    }

//    printf("%d", read_buffer_len);
//    printf("\n");
//    printf("%d", nread);
//    printf("\n");

    return((asynStatus)status);
}


// TODO


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

    /* Reject any call to the detector if it is running */
    getIntegerParam(ADAcquire, &acquire);
    if ((function != ADAcquire) & acquire) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: detector is busy\n", driverName, functionName);
        return asynError;
    }

    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (value & !acquire) {
            status |= startCapture();
        } else if (!value & acquire) {
            status |= stopCapture();
        }
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

    return((asynStatus)status);
}

/** Called when asyn clients call pasynFlaot64->write().
  * This function performs actions for some parameters, including ADAcquire, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus visarCamera::writeFloat64(asynUser *pasynUser, epicsFloat64 value){
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

    return((asynStatus)status);
}

/*
asynStatus visarCamera::readInt32(	asynUser *	pasynUser, epicsInt32	* pValueRet )
{
    static const char	*	functionName	= "readInt32";
    const char			*	reasonName		= "unknownReason";
    getParamName( 0, pasynUser->reason, &reasonName );

        printf(	"%s: Reason %d %s\n", functionName, pasynUser->reason, reasonName );


    if ( pasynUser->reason == SDAppStatus    ){
        *pValueRet=56;
        setIntegerParam( SDAppStatus, *pValueRet );
    }

    // Call base class
    // asynStatus	status	= asynPortDriver::readInt32( pasynUser, pValueRet );
    asynStatus	status	= ADDriver::readInt32( pasynUser, pValueRet );
    return((asynStatus)status);
}
*/


/** Code for iocsh registration */

static const iocshArg visarCameraConfigArg0 = {"Port name", iocshArgString};
static const iocshArg visarCameraConfigArg1 = {"Control asyn port name", iocshArgString};
static const iocshArg visarCameraConfigArg2 = {"Data asyn port name", iocshArgString};
static const iocshArg visarCameraConfigArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg visarCameraConfigArg4 = {"maxMemory", iocshArgInt};
static const iocshArg visarCameraConfigArg5 = {"priority", iocshArgInt};
static const iocshArg visarCameraConfigArg6 = {"stackSize", iocshArgInt};
static const iocshArg *const visarCameraConfigArgs[] = {&visarCameraConfigArg0,
                                                        &visarCameraConfigArg1,
                                                        &visarCameraConfigArg2,
                                                        &visarCameraConfigArg3,
                                                        &visarCameraConfigArg4,
                                                        &visarCameraConfigArg5,
                                                        &visarCameraConfigArg6,};


static const iocshFuncDef configvisarcamera = {"visarCameraConfig", 7, visarCameraConfigArgs};

static void configvisarcameraCallFunc(const iocshArgBuf *args) {
    visarCameraConfig(args[0].sval, args[1].sval, args[2].sval,
                      args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}

static void visarCameraRegister(void) {

    iocshRegister(&configvisarcamera, configvisarcameraCallFunc);
}

extern "C" {
epicsExportRegistrar(visarCameraRegister);
}





