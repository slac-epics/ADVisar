#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>


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
#include <aSubRecord.h>


#define DRIVER_VERSION      0
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

#define SIZE_X 1344
#define SIZE_Y 1024
#define BPP 2

// Data port timeout
#define VC_D_TIMEOUT 5

#define VisarFrameReadyString "VC_FRAME_READY"
#define VisarStartAcquisitionString "VC_START_ACQ"
#define VisarStopAcquisitionString "VC_STOP_ACQ"
#define VisarFrameRateString "VC_FRAME_RATE"


using namespace std;


static const char *driverName = "visarCamera";

class visarCamera : public ADDriver {
public:

    visarCamera(const char *portName, const char *controlPortName,
                const char *dataPortName,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    virtual ~visarCamera() {};

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    virtual void report(FILE *fp, int details);

    //These should be private but are called from C callback functions, must be public.
    void imageGrabTask();

    void shutdown();

protected:

    // Visar Camera parameters
    int VisarFrameReady;
#define FIRST_SD_PARAM VisarFrameReady
    int StartAcquisition;
    int StopAcquisition;
    int FrameRate;
#define LAST_SD_PARAM FrameRate

private:

    const char *controlPortName_;
    const char *dataPortName_;
    asynUser *pasynUserData_;
    char rawFrame_[SIZE_X * SIZE_Y * BPP];
    NDArray *pRaw_;
    epicsEventId startEventId_;
    epicsEventId NewFrame;

    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus startCapture();
    asynStatus stopCapture();
    asynStatus grabImage();
    asynStatus waitForFrameNotification();
    asynStatus getRawFrame();
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


/** Constructor for visarCamera driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data,
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
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

    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId_) {
        printf("%s:%s epicsEventCreate failure for run event\n", driverName, functionName);
        return;
    }
    NewFrame = epicsEventCreate(epicsEventEmpty);
    if (!this->NewFrame) {
        printf("%s:%s epicsEventCreate failure for new frame event\n", driverName, functionName);
        return;
    }

    // Set static parameters
    status |= setStringParam(ADManufacturer, "Hamamatsu");
    status |= setIntegerParam(ADMaxSizeX, SIZE_X);
    status |= setIntegerParam(ADMaxSizeY, SIZE_Y);
    status |= setIntegerParam(ADMinX, 0);
    status |= setIntegerParam(ADMinY, 0);
    status |= setIntegerParam(NDDataType, NDUInt16);
    status |= setIntegerParam(NDBitsPerPixel, 16);

    createParam(VisarFrameReadyString, asynParamInt32, &VisarFrameReady);
    createParam(VisarStartAcquisitionString, asynParamInt32, &StartAcquisition);
    createParam(VisarStopAcquisitionString, asynParamInt32, &StopAcquisition);
    createParam(VisarFrameRateString, asynParamFloat64, &FrameRate);

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
                      epicsThreadPriorityHigh + 9,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      imageGrabTaskC, this);

}

/** Connects to camera.
 *  Connects to the data port.
 *  Starts the Visar camera app*/
asynStatus visarCamera::connectCamera(void) {

    static const char *functionName = "visarCamera::connectCamera";
    int status = asynSuccess;
    size_t cnt = 0;
    int eom = 0, off = 0;
    char buf[256], *s, *t;
    asynUser *pasynUserCtrl;

    status |= pasynOctetSyncIO->connect(dataPortName_, 0, &pasynUserData_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserData_->errorMessage);
        return ((asynStatus) status);
    }
    
    /* We start the app here directly! */
    status |= pasynOctetSyncIO->connect(controlPortName_, 0, &pasynUserCtrl, NULL);
    status |= pasynOctetSyncIO->write(pasynUserCtrl, "AppStart()\r", 11, 1.0, &cnt);
    for (;;) {
        pasynOctetSyncIO->read(pasynUserCtrl, &buf[off], sizeof(buf) - off, 1.0, &cnt, &eom);
        buf[cnt] = 0;
        for (s = t = buf; *t; t++) {
            if (*t == '\r') {
                *t = 0;
                if (*s == '0')  // "0,AppStart", presumably!
                    break;
                s = t + 1;
            }
        }
        if (*s == '0')
            break;
        off = 0;
        if (*s) {
            // Sigh.  A partial line.
            for (t = buf; *s; *t++ = *s++)
                off++;
        }
    }
    status |= pasynOctetSyncIO->disconnect(pasynUserCtrl);
    return ((asynStatus) status);
}

/** Disconnects to camera.
 *  Stops the Visar camera app*/
asynStatus visarCamera::disconnectCamera() {
    // static const char *functionName = "visarCamera::disconnectCamera";

    int status = asynSuccess;
    /* We don't really want to stop the app!!! This doesn't work anyway... */
    return ((asynStatus) status);
}

/** Starts acquisition and activates live monitor. */
asynStatus visarCamera::startCapture() {

    // static const char *functionName = "startCapture";

    /* Start the camera transmission... */
    setIntegerParam(ADNumImagesCounter, 0);

    int status = asynSuccess;

    int startAcquisition;
    getIntegerParam(StartAcquisition, &startAcquisition);
    status |= setIntegerParam(StartAcquisition, !startAcquisition);

    status |= setIntegerParam(ADAcquire, 1);

    epicsEventSignal(startEventId_);

    return ((asynStatus) status);
}

/** Stops acquisition and deactivates live monitor. */
asynStatus visarCamera::stopCapture() {

    // static const char *functionName = "stopCapture";

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

        // Grab a new frame and *set* the status!
        status = grabImage();
        if (status != asynSuccess) {
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
        setIntegerParam(NDBitsPerPixel, 16);

        /* Put the frame number and time stamp into the buffer */
        pRaw_->uniqueId = imageCounter;
        //pRaw_->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
        updateTimeStamp(&pRaw_->epicsTS);
        pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;

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
        //if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
        //    status |= stopCapture();
        // }
        callParamCallbacks();
    }

}

/** Get a new frame. */
asynStatus visarCamera::grabImage() {
    static const char *functionName = "grabImage";

    int status = asynSuccess;

    size_t dims[3];
    size_t dataSize;

    int nDims;
    int acquire;

    /* Unlock the driver while we wait for a new image to be ready */
    unlock();

    /* Wait until we receive a frame notification */
    status |= waitForFrameNotification();

    if (status != asynSuccess) {
        lock();
        return asynError;
    }

    status |= getRawFrame();

    /* Check if the aquisition was stopped */
    status |= getIntegerParam(ADAcquire, &acquire);
    if (!acquire) {
        lock();
        return asynError;
    }

    lock();

    dataSize = SIZE_X * SIZE_Y * BPP;

    status |= setIntegerParam(ADSizeX, SIZE_X);
    status |= setIntegerParam(ADSizeY, SIZE_Y);

    status |= setIntegerParam(NDArraySizeX, SIZE_X);
    status |= setIntegerParam(NDArraySizeY, SIZE_Y);
    status |= setIntegerParam(NDArraySize, (int) dataSize);
    status |= setIntegerParam(NDDataType, NDUInt16);
    status |= setIntegerParam(NDColorMode, NDColorModeMono);
    callParamCallbacks();

    nDims = 2;
    dims[0] = SIZE_X;
    dims[1] = SIZE_Y;
    pRaw_ = pNDArrayPool->alloc(nDims, dims, NDUInt16, 0, NULL);

    if (!pRaw_) {
        /* If we didn't get a valid buffer from the NDArrayPool we must abort
         * the acquisition as we have nowhere to dump the data...       */
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
                  driverName, functionName, portName);
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
        return (asynError);
    }

    memcpy(pRaw_->pData, rawFrame_, dataSize);

    pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec / 1e9;

    /* Get any attributes that have been defined for this driver */
    getAttributes(pRaw_->pAttributeList);

    /* Change the status to be readout... */
    status |= setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();

    return ((asynStatus) status);
}

/** Wait for a new frame notification. */
asynStatus visarCamera::waitForFrameNotification() {

    static const char *functionName = "waitForFrameNotification";

    int acquire;
    int frame;

    while (1) {
        if (epicsEventWaitWithTimeout(NewFrame, 0.2) == epicsEventWaitOK) {
            getIntegerParam(VisarFrameReady, &frame);
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s New frame ready: %d\n",
                      driverName, functionName,frame);
            return asynSuccess;
        }

        /* Check if the aquisition was stopped */
        getIntegerParam(ADAcquire, &acquire);
        if (!acquire) {
            printf("Acquisition stopped during waiting time \n");
            return asynError;
        }
    }
}

/** Ask for a new frame data and get it from the data port. */
asynStatus visarCamera::getRawFrame() {

    const char *functionName = "getRawFrame";

    size_t nread, nwrite;
    int status = asynSuccess;
    int eomReason;
    asynUser *pasynUserCtrl;

    status |= pasynOctetSyncIO->connect(controlPortName_, 0, &pasynUserCtrl, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for data socket, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserData_->errorMessage);
        return ((asynStatus) status);
    }

    pasynOctetSyncIO->flush(pasynUserData_);

    status |= pasynOctetSyncIO->write(pasynUserCtrl, "ImgDataGet(Current,Data)\r",
                                      25, 1.0, &nwrite);

    size_t read_buffer_len = SIZE_X * SIZE_Y * BPP;

    status |= pasynOctetSyncIO->read(pasynUserData_, rawFrame_, read_buffer_len, 2.0, &nread, &eomReason);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: read frame error, nread=%d, eomReason=%d, status=%d!\n",
                  driverName, functionName, (int)nread, eomReason, status);
        //pasynOctetSyncIO->flush(pasynUserData_);
    }

    status |= pasynOctetSyncIO->disconnect(pasynUserCtrl);
    return ((asynStatus) status);
}



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
    } else if (function == VisarFrameReady) {
        getIntegerParam(function, &frame);
        if (value != frame) {
            if (acquire)
                epicsEventSignal(NewFrame);
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

    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setDoubleParam(function, value);

    /* Update any changed parameters */
    status |= callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%f\n",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%f\n",
                  driverName, functionName, function, value);

    return ((asynStatus) status);
}


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
                                                        &visarCameraConfigArg6,
                                                        };

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



/** Code to read the calibration file
 */
double sciToDub(const string &str) {

    stringstream ss(str);
    double d = 0;
    ss >> d;

    if (ss.fail()) {
        string s = "Unable to format ";
        s += str;
        s += " as a number!";
        throw (s);
    }

    return (d);
}

static double get_calibration(char *path, char *time_range, double *cal,
                              double *cal_factor, char *cal_string) {

    ifstream infile;
    char cNum[256];

    string runits;
    double c0, c1, c2;
    double factor = 1;

    vector <vector <string> > data;

    infile.open(path, ifstream::in);
    if (infile.is_open()) {
        while (infile.good()) {
            infile.getline(cNum, 256);

            istringstream ss(cNum);

            vector <string> record;

            while (ss.good()) {
                ss.getline(cNum, 256, ',');
                record.push_back(cNum);
            }
            data.push_back(record);
        }
        infile.close();
        
        infile.open(path);
        
        stringstream strStream;
        strStream << infile.rdbuf();
        string str = strStream.str();
        strcpy(cal_string, str.c_str());
        
        infile.close();
  
        for (unsigned int k = 3; k < data.size(); k++) {
            vector <string> time_cal = data[k];

            if (time_cal[0].compare(time_range) == 0) {

                runits = time_cal[1];
                c0 = sciToDub(time_cal[3]);
                c1 = sciToDub(time_cal[4]);
                c2 = sciToDub(time_cal[5]);

                if (runits.compare("ns") == 0) factor = 1e-9;
                else if (runits.compare("us") == 0) factor = 1e-6;
                else if (runits.compare("ms") == 0) factor = 1e-3;
                else if (runits.compare("s") == 0) factor = 1;

                cout << "Found in Calibration file  " << time_cal[0] << "\t" << runits << "\t" << factor << "\t" << c0
                     << "\t" << c1 << "\t" << c2 << "\n";

                cal[0] = 0;
                for (int n = 1; n < 1344; n++) {
                    cal[n] = cal[n - 1] + (c0 + c1 * n + c2 * n * n) * factor;
                }
                double xy_sum = 0;
                double xx_sum = 0;
                for (int x = 1; x < 1344; x++) {
                    xy_sum = xy_sum+  ( x * cal[x] );
                    xx_sum = xx_sum + ( x * x );
                }

                cal_factor[0] = xy_sum / xx_sum;

                cout << "Linear Factor: " << cal_factor[0] << " sec/pixel\n";
                return 0;
            }
        }
        printf("Time range not found in calibration file.");

        return 1;
    } else {
        cout << "Error opening file: " << path << "\n";
        return 1;
    }


}

static long read_calibration_file_subprocess(aSubRecord *prec) {

    char *path, *time_range;

    path       = (char *) prec->a;
    time_range = (char *) prec->b;

    double *vala = (double *) prec->vala;
    double *valb = (double *) prec->valb;
    char   *valc = (char *)   prec->valc;
    
    get_calibration(path, time_range, vala, valb, valc);

    return 0; /* process output links */
}

epicsRegisterFunction(read_calibration_file_subprocess);
