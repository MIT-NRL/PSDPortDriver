#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

#include <epicsExport.h>
#include <osiSock.h>
#include <sys/cdefs.h>
#include <sys/param.h>

#include "asynDriver.h"
#include "asynParamType.h"
#include "include/portable_endian.h"
#include "include/tEndian.h"
#include "psdPortDriver.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#define PRINT_EVENTS 0

static const char *driverName = "psdPortDriver";

void threadReadEventLoop(void *drvPtr) {
    psdPortDriver *drv = (psdPortDriver *)drvPtr;
    drv->readEventLoop();
}

psdPortDriver::psdPortDriver(const char *portName, const char *address,
                             const char *tcpPort, const char *udpPort)
    : asynPortDriver( // clang-format off
        portName,                                                               /* portName */
        1,                                                                      /* maxAddr */
        asynInt32Mask | asynInt64Mask | asynFloat64Mask
            | asynInt32ArrayMask | asynInt64ArrayMask | asynDrvUserMask,        /* Interface mask */
        asynInt32Mask | asynInt64Mask | asynFloat64Mask
            | asynInt32ArrayMask | asynInt64ArrayMask,                          /* Interrupt mask */
        ASYN_CANBLOCK,                                                          /* asynFlags */
        1,                                                                      /* Autoconnect */
        0,                                                                      /* Default priority */
        0                                                                       /* Default stack size*/
      ) { // clang-format on
    tcpSocket = INVALID_SOCKET;
    udpSocket = INVALID_SOCKET;
    detTCPAddrinfo = NULL;
    detUDPAddrinfo = NULL;

    isConnected = false;
    detAddr = epicsStrDup(address);
    detTCPPort = epicsStrDup(tcpPort);
    detUDPPort = epicsStrDup(udpPort);

    // Create the epicsEvents for signaling aquisition start / stop aquiring
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!startEventId_) {
        printf("%s:%s epicsEventCreate failure for start event\n", driverName,
               __func__);
        return;
    }
    stopEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!stopEventId_) {
        printf("%s:%s epicsEventCreate failure for stop event\n", driverName,
               __func__);
        return;
    }

    createParam(P_AcquireString, asynParamInt32, &P_Acquire);
    createParam(P_AcquireTimeString, asynParamFloat64, &P_AcquireTime);
    createParam(P_NumBinsString, asynParamInt32, &P_NumBins);
    createParam(P_CountsString, asynParamInt32Array, &P_Counts);
    createParam(P_TotalCountsString, asynParamInt64Array, &P_TotalCounts);
    createParam(P_LiveCountsString, asynParamInt32Array, &P_LiveCounts);
    createParam(P_LiveTotalCountsString, asynParamInt64Array,
                &P_LiveTotalCounts);

    // Create the thread that runs the read event loop
    asynStatus status =
        (asynStatus)(epicsThreadCreate(
                         "PSD_ReadEventLoop", epicsThreadPriorityMedium,
                         epicsThreadGetStackSize(epicsThreadStackMedium),
                         (EPICSTHREADFUNC)::threadReadEventLoop, this) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, __func__);
        return;
    }
}

psdPortDriver::~psdPortDriver() {
    this->freeNetworking();

    free(this->detAddr);
    free(this->detTCPPort);
    free(this->detUDPPort);
}

/** Called when asyn clients call pasynInt32->write().
 * This function sends a signal to the simTask thread if the value of P_Run has
 * changed. For all parameters it sets the value in the parameter library and
 * calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write.
 */
asynStatus psdPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    // Fetch the parameter string name for possible use in debugging
    const char *paramName;
    getParamName(function, &paramName);

    // Update values
    int acquiring;
    getIntegerParam(P_Acquire, &acquiring);

    if (function == P_Acquire) {
        if (value && !acquiring) {
            // Send an event to wake up the acquisition run loop
            // Won't start running until lock is released
            epicsEventSignal(startEventId_);
        }
        if (!value && acquiring) {
            // Stop aquisition
            epicsEventSignal(stopEventId_);
        }
    } else {
        /* All other parameters just get set in parameter list, no need to
         * act on them here */
    }

    // Set the parameter in the parameter library.
    status = (asynStatus)setIntegerParam(function, value);

    /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();

    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s:%s: status=%d, function=%d, name=%s, value=%d",
                      driverName, __func__, status, function, paramName, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, name=%s, value=%d\n", driverName,
                  __func__, function, paramName, value);
    }

    printf("%s:%s: function=%d, name=%s, value=%d\n", driverName, __func__,
           function, paramName, value);
    return status;
}

/** Called when asyn clients call pasynInt32Array->read().
 * Returns the value of the P_Waveform or P_TimeBase arrays.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Pointer to the array to read.
 * \param[in] nElements Number of elements to read.
 * \param[out] nIn Number of elements actually read.
 */
asynStatus psdPortDriver::readInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                         size_t nElements, size_t *nIn) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    epicsTimeStamp timeStamp;

    getTimeStamp(&timeStamp);
    pasynUser->timestamp = timeStamp;

    size_t ncopy = std::min(nElements, this->counts_.size());

    if (function == P_Counts) {
        std::copy_n(this->counts_.begin(), ncopy, value);
        *nIn = ncopy;
    } else if (function == P_LiveCounts) {
        std::copy_n(this->counts_.begin(), ncopy, value);
        *nIn = ncopy;
    }

    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s:%s: status=%d, function=%d", driverName, __func__,
                      status, function);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: function=%d\n",
                  driverName, __func__, function);
    }
    return status;
}

/** Connects driver to device */
asynStatus psdPortDriver::connect(asynUser *pasynUser) {
    int addr;
    asynStatus status = getAddress(pasynUser, &addr);
    if (status != asynSuccess) {
        return status;
    }

    // Clean up everything
    this->freeNetworking();

    // Create a TCP and UDP socket
    struct addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(detAddr, detTCPPort, &hints, &detTCPAddrinfo) != 0) {
        asynPrint(
            pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but getaddrinfo failed (TCP)\n",
            driverName, __func__);
        return asynError;
    }

    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(detAddr, detUDPPort, &hints, &detUDPAddrinfo) != 0) {
        asynPrint(
            pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but getaddrinfo failed (UDP)\n",
            driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    tcpSocket = epicsSocketCreate(detTCPAddrinfo->ai_family,
                                  detTCPAddrinfo->ai_socktype, 0);
    if (tcpSocket == INVALID_SOCKET) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s error attempt to connect but socket creation failed "
                  "(TCP)\n",
                  driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    udpSocket = epicsSocketCreate(detUDPAddrinfo->ai_family,
                                  detUDPAddrinfo->ai_socktype, 0);
    if (udpSocket == INVALID_SOCKET) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s error attempt to connect but socket creation failed "
                  "(UDP)\n",
                  driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    if (::connect(tcpSocket, detTCPAddrinfo->ai_addr,
                  detTCPAddrinfo->ai_addrlen) == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s error attempt to connect but connecting to socket "
                  "failed (TCP)\n",
                  driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    if (::connect(udpSocket, detUDPAddrinfo->ai_addr,
                  detUDPAddrinfo->ai_addrlen) == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s error attempt to connect but connecting to socket "
                  "failed (UDP)\n",
                  driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    // // Set socket timeout for recv
    // struct timeval tv;
    // tv.tv_sec = 1;
    // tv.tv_usec = 0;
    // setsockopt(tcpSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv,
    //            sizeof tv);

    isConnected = true;

    int setupStatus = this->setup();
    if (setupStatus < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s error during NEUNET setup\n", driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    pasynManager->exceptionConnect(pasynUser);
    return asynSuccess;
}

asynStatus psdPortDriver::disconnect(asynUser *pasynUser) {
    if (isConnected) {
        this->teardown();
    }

    this->freeNetworking();
    return asynPortDriver::disconnect(pasynUser);
}

void psdPortDriver::freeNetworking() {
    if (detTCPAddrinfo != NULL)
        freeaddrinfo(detTCPAddrinfo);
    if (detUDPAddrinfo != NULL)
        freeaddrinfo(detUDPAddrinfo);
    if (tcpSocket != INVALID_SOCKET)
        epicsSocketDestroy(tcpSocket);
    if (udpSocket != INVALID_SOCKET)
        epicsSocketDestroy(udpSocket);

    detTCPAddrinfo = NULL;
    detUDPAddrinfo = NULL;
    tcpSocket = INVALID_SOCKET;
    udpSocket = INVALID_SOCKET;

    isConnected = false;
}

/* UDP NETWORKING */

#define UDP_CMD_R 0xC
#define UDP_CMD_W 0x8

struct UDPMessageHeader {
    uint8_t type : 4; /* lower 4 bits */
    uint8_t ver : 4;  /* upper 4 bits */

    uint8_t flag : 4; /* lower 4 bits */
    uint8_t cmd : 4;  /* upper 4 bits */

    uint8_t id;

    uint8_t dataLength;
    beuint32 address;
} __attribute__((packed));

/**
 * Reads/writes data from and to a NEUNET register.
 * Returns the number of bytes written to `readBuf`, or -1 if something failed.
 *
 * \param[in] address Address of the register to write to.
 * \param[in] data Data to write (set to NULL to read).
 * \param[in] dataLength Length of `data` or how many bytes to read.
 * \param[out] readBuf Buffer to copy read bytes to.
 * \param[in] readBufSize Size of `readBuf`.
 */
int psdPortDriver::sendNEUNET(uint32_t address, const char *data,
                              uint8_t dataLength, char *readBuf,
                              size_t readBufSize) {
    if (!this->isConnected) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s error: not connected\n", driverName, __func__);
        return -1;
    }

    static uint8_t updIdCounter = 0;
    uint8_t msg_id = (updIdCounter++);

    // Construct Message
    struct UDPMessageHeader header;
    header.ver = 0xF;
    header.type = 0xF;
    header.cmd = (data != NULL) ? UDP_CMD_W : UDP_CMD_R;
    header.flag = 0x0;
    header.id = msg_id;
    header.dataLength = dataLength;
    header.address = address;

    int msgLength = sizeof(header) + dataLength;
    char *msg = (char *)malloc(msgLength);

    memcpy(msg, &header, sizeof(header));
    memcpy(msg + sizeof(header), data, dataLength);

    // Send Message
    int sentBytes = send(this->udpSocket, msg, msgLength, 0);
    free(msg);

    if (sentBytes == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s error: sending failed\n", driverName, __func__);
        return -1;
    }

    // Get Response
    char buf[256 + sizeof(UDPMessageHeader)];
    int readBytes = recv(this->udpSocket, buf, sizeof(buf), 0);

    if (readBytes < (int)sizeof(UDPMessageHeader)) {
        return -1;
    }

    auto recvHeader = reinterpret_cast<UDPMessageHeader *>(buf);
    if (recvHeader->id != msg_id) {
        // Incorrect ID
        return -1;
    }

    if (recvHeader->flag != 0b1000) {
        // Either not an acknowledge package, or there was an error
        return -1;
    }

    // Copy out the data
    if (readBuf != NULL) {
        memcpy(readBuf, buf + sizeof(UDPMessageHeader),
               std::min(readBufSize, (size_t)recvHeader->dataLength));
        return std::min(readBufSize, (size_t)recvHeader->dataLength);
    }

    return 0;
}

#define NEUNET_ADDR_TIMEMODE   0x18A
#define NEUNET_ADDR_DEVICETIME 0x190
#define NEUNET_ADDR_RW         0x187
#define NEUNET_ADDR_STATUS_LO  0x189
#define NEUNET_ADDR_RESOLUTION 0x1B4
#define NEUNET_ADDR_MODE       0x1B5

/** Set up all the necessary NEUNET registers for operation */
int psdPortDriver::setup() {
    // sendNEUNET returns a negative value in case of an error.
    // By oring all status values together, we can just check at the very end if
    // the value is negative to determine if any of the sendNEUNET commands
    // failed.
    int status = 0;

    // Set time mode to be 32 bit resolution
    const unsigned char timeMode32Bit[] = {0x80};
    status |= this->sendNEUNET(NEUNET_ADDR_TIMEMODE, (char *)timeMode32Bit,
                               sizeof(timeMode32Bit), NULL, 0);

    // Send current time to device
    psdTime32_t currentTime = epicsTimeToPSDTime32_t(epicsTime::getCurrent());
    unsigned char currentTimeData[7] = {0};
    memcpy(currentTimeData, &currentTime, sizeof(currentTime));
    status |= this->sendNEUNET(NEUNET_ADDR_DEVICETIME, (char *)currentTimeData,
                               sizeof(currentTimeData), NULL, 0);

    // Event memory read mode
    const unsigned char evenMemoryReadMode[] = {0x00};
    status |= this->sendNEUNET(NEUNET_ADDR_RW, (char *)evenMemoryReadMode,
                               sizeof(evenMemoryReadMode), NULL, 0);

    // Set resolution to 14 bit
    const unsigned char resolution14Bit[] = {0x8A};
    status |= this->sendNEUNET(NEUNET_ADDR_RESOLUTION, (char *)resolution14Bit,
                               sizeof(resolution14Bit), NULL, 0);

    // Switch to oneway mode (disable handshake)
    const unsigned char oneWayMode[] = {0x80};
    status |= this->sendNEUNET(NEUNET_ADDR_MODE, (char *)oneWayMode,
                               sizeof(oneWayMode), NULL, 0);

    if (status < 0) {
        return -1;
    }

    return 0;
}

/** Resets NEUNET registers */
int psdPortDriver::teardown() {
    // Switch back to handshake mode
    const char handshakeMode[] = {0x00};
    this->sendNEUNET(NEUNET_ADDR_MODE, handshakeMode, sizeof(handshakeMode),
                     NULL, 0);

    return 0;
}

/**
 * Flush the NEUNET FIFO and all TCP data.
 * This can be useful to get rid of stale data.
 */

void psdPortDriver::flushNEUNET() {
    // Tell NEUNET to flush FIFO
    const char flushFIFO[] = {0x40};
    this->sendNEUNET(NEUNET_ADDR_STATUS_LO, flushFIFO, sizeof(flushFIFO), NULL,
                     0);

    // Flush the TCP read buffer
    const int bufSize = 1024;
    char buf[bufSize];

    // Set socket to non-blocking mode
#if _WIN32
    u_long mode = 1; // non-zero sets non-blocking mode
    ioctlsocket(this->tcpSocket, FIONBIO, &mode);
#else
    int flags = fcntl(this->tcpSocket, F_GETFL, 0);
    fcntl(this->tcpSocket, F_SETFL, flags | O_NONBLOCK);
#endif

    // Read all available data
    while (recv(this->tcpSocket, buf, bufSize, 0) > 0) {
    }

    // Set socket back to blocking mode
#if _WIN32
    mode = 0; // zero sets blocking mode
    ioctlsocket(this->tcpSocket, FIONBIO, &mode);
#else
    fcntl(this->tcpSocket, F_SETFL, flags);
#endif
}

/* TCP Networking */

enum class TCPEventType {
    neutronData12 = 0x5a,
    neutronData14 = 0x5f,
    triggerId = 0x5b,
    triggerIdT0Sync = 0x51,
    instrumentTime30 = 0x5c,
    instrumentTime32 = 0x6c,
};

void psdPortDriver::readEventLoop() {
    enum class AcquisitionState { stopped, starting, acquiring, stopping };

    AcquisitionState state = AcquisitionState::stopped;
    double acquireTime;
    int numBins;
    epicsTime startTime;

    int status = asynSuccess;
    char eventBuffer[8];

    auto notConnectedHandler = [this, &status, &state]() {
        // Try to reconnect once, otherwise stop acquisition without returning
        // new data.
        this->disconnect(this->pasynUserSelf);
        status = this->connect(this->pasynUserSelf);
        if (status != asynSuccess) {
            state = AcquisitionState::stopped;
            setIntegerParam(P_Acquire, 0);
            callParamCallbacks();
        }
    };

    this->lock();
    while (true) {
    loopStart:
        switch (state) {
        case AcquisitionState::stopped: {
            // If we are not acquireing then wait for a semaphore that is given
            // when acquisition is started
            this->unlock();
            status = epicsEventWait(startEventId_);
            this->lock();
            state = AcquisitionState::starting;

            // Clear out old counts
            std::fill(totalCounts_.begin(), totalCounts_.end(), 0);
            std::fill(counts_.begin(), counts_.end(), 0);
            this->flushNEUNET();
            this->realignTCP();

            // Reset Live Counts
            size_t countsSize = numBins * PSD_NUM_DETECTORS;
            doCallbacksInt32Array(this->counts_.data(), countsSize,
                                  P_LiveCounts, 0);
            callParamCallbacks();

            // Read parameters
            getDoubleParam(P_AcquireTime, &acquireTime);
            getIntegerParam(P_NumBins, &numBins);
        } break;

        case AcquisitionState::starting:
        case AcquisitionState::acquiring: {
            // Check if we need to stop acquiring
            status = epicsEventTryWait(stopEventId_);
            if (status == epicsEventOK) {
                state = AcquisitionState::stopping;
                goto loopStart;
            }
        } break;

        case AcquisitionState::stopping: {
            // Notify EPICS of new counts data
            size_t countsSize = numBins * PSD_NUM_DETECTORS;
            doCallbacksInt64Array(this->totalCounts_.data(),
                                  this->totalCounts_.size(), P_TotalCounts, 0);
            doCallbacksInt64Array(this->totalCounts_.data(),
                                  this->totalCounts_.size(), P_LiveTotalCounts,
                                  0);
            doCallbacksInt32Array(this->counts_.data(), countsSize, P_Counts,
                                  0);
            doCallbacksInt32Array(this->counts_.data(), countsSize,
                                  P_LiveCounts, 0);
            callParamCallbacks();

            // Set acquire back to done after the other PV have been updated
            setIntegerParam(P_Acquire, 0);
            callParamCallbacks();
            state = AcquisitionState::stopped;
            goto loopStart;
        }
        }

        if (!this->isConnected) {
            notConnectedHandler();
            continue;
        }

        this->unlock();
        // TODO: Add some kind of timeout...
        int readBytes = readEvent(eventBuffer);
        this->lock();

        if (readBytes == 0) {
            // Reading zero bytes means the TCP connections is closed
            printf("TCP Disconnected! Trying to reconnect once...\n");
            notConnectedHandler();
            continue;
        }

        if (readBytes != 8) {
            printf("Read Event Loop error\n");
            this->unlock();
            epicsThreadSleep(0.1);
            this->lock();
            continue;
        }

        // Event handling
        PSDEventData event = parseEventData(eventBuffer);
        switch (event.type) {
        case PSDEventType::neutron: {
            if (state != AcquisitionState::acquiring)
                break;

            NeutronData n = event.neutron;
            double tof = (double)n.triggerOffset * 0.000000025;

            // Update counts
            if (std::isnan(n.position))
                break;

            int binIndex = static_cast<int>(std::floor(n.position * numBins));
            int offset = numBins * n.detector;

            if (binIndex < 0) {
                this->counts_[offset]++;
            } else if (binIndex >= numBins) {
                this->counts_[offset + numBins - 1]++;
            } else {
                this->counts_[offset + binIndex]++;
            }

            this->totalCounts_[n.detector]++;
        } break;

        case PSDEventType::triggerId:
            break;

        case PSDEventType::instrumentTime: {
            epicsTime time = event.instrumentTime;
            if (state == AcquisitionState::starting) {
                startTime = time;
                state = AcquisitionState::acquiring;
            } else if (acquireTime > 0) {
                double timeDelta = time - startTime;
                if (timeDelta >= acquireTime) {
                    state = AcquisitionState::stopping;
                }
            }

            // TODO: Determine how often to actually update the live counts
            callParamCallbacks();
            doCallbacksInt64Array(this->totalCounts_.data(),
                                  this->totalCounts_.size(), P_LiveTotalCounts,
                                  0);
            doCallbacksInt32Array(this->counts_.data(),
                                  numBins * PSD_NUM_DETECTORS, P_LiveCounts, 0);
        } break;

        case PSDEventType::error: {
            // This should only be the case if there is an error with the
            // alignment of our data -> attempt to fix it
            this->realignTCP();
        } break;
        }

#if PRINT_EVENTS
        const char *eventTypeStr;
        char eventDescrStr[256] = {0};
        char hexStr[32] = {0};

        for (int i = 0; i < 8; i++)
            sprintf(&hexStr[3 * i], "%02X ", eventBuffer[i]);
        hexStr[3 * 8 - 1] = 0; // Remove trailing space

        switch (event.type) {
        case PSDEventType::neutron: {
            NeutronData n = event.neutron;
            double tof = (double)n.triggerOffset * 0.000000025;

            eventTypeStr = "[Neutron Event]";
            sprintf(eventDescrStr, "pos=%f  det=%d  tof=%f", n.position,
                    n.detector, tof);
        } break;

        case PSDEventType::triggerId: {
            TriggerId t = event.triggerId;

            eventTypeStr = "[Trigger Event]";
            sprintf(eventDescrStr, "id=%ld  module=%d  crate=%d", t.triggerId,
                    t.module, t.crate);
        } break;

        case PSDEventType::instrumentTime: {
            epicsTime time = event.instrumentTime;

            eventTypeStr = "[Instrument Time]";
            time.strftime(eventDescrStr, 256, "%Y-%m-%d %H:%M:%S.%06f");
        } break;

        case PSDEventType::error: {
            eventTypeStr = "[Event Error]";
            eventDescrStr[0] = '-';
            eventDescrStr[1] = 0;
        } break;
        }

        printf("%-20s %-40s (%s)\n", eventTypeStr, eventDescrStr, hexStr);
#endif
    }
}

/**
 * Read / receives an event from NEUNET.
 * Events can include neutron data, trigger id and instrument time.
 * All events are 8 bytes long. If receiving data fails, returns -1.
 * \param[out] buf Buffer to which the received data is written to.
 *                 Must be at least 8 bytes long.
 */
int psdPortDriver::readEvent(char *buf) {
    if (!isConnected)
        return 0;

    int bytesRead = recv(this->tcpSocket, buf, 8, MSG_WAITALL);
    if (bytesRead != 8)
        printf("Error in readEvent - Read less than 8 bytes (%d)\n", bytesRead);
    return bytesRead;
}

/**
 * Peek the TCP data, and if it appears to be offset, realign it.
 * This should fix issues in case of missing data.
 */
void psdPortDriver::realignTCP() {
    char buf[32 - 1];
    int bytesRead =
        recv(this->tcpSocket, buf, sizeof(buf), MSG_WAITALL + MSG_PEEK);

    if (bytesRead != sizeof(buf)) {
        return;
    }

    for (int offset = 0; offset < 8; offset++) {
        int validHeaders = 0;
        int invalidHeaders = 0;

        for (int i = offset; i < (int)sizeof(buf); i += 8) {
            switch ((TCPEventType)buf[i]) {
            case TCPEventType::neutronData12:
            case TCPEventType::neutronData14:
            case TCPEventType::triggerId:
            case TCPEventType::triggerIdT0Sync:
            case TCPEventType::instrumentTime30:
            case TCPEventType::instrumentTime32:
                validHeaders += 1;
                break;
            default:
                invalidHeaders += 1;
                break;
            }
        }

        if (validHeaders && !invalidHeaders) {
            // We found an offset, so now we can advance the TCP data buffer
            // by the ammount corresponding to the offset we found.
            recv(this->tcpSocket, buf, offset, MSG_WAITALL);
            return;
        }
    }
}

PSDEventData parseEventData(char *buf) {
    union {
        struct {
            uint64_t pr : 12;   // Pulse Right
            uint64_t pl : 12;   // Pulse Left
            uint64_t p : 8;     // Detector Number
            uint64_t t : 24;    // Detector Time after Trigger Pulse [25ns]
            uint64_t event : 8; // Event Type
        } neutronData12;

        struct {
            uint64_t pr : 14;   // Pulse Right
            uint64_t pl : 14;   // Pulse Left
            uint64_t p : 4;     // Detector Number
            uint64_t t : 24;    // Detector Time after Trigger Pulse [25ns]
            uint64_t event : 8; // Event Type
        } neutronData14;

        struct {
            uint64_t k : 40;    // Trigger ID
            uint64_t m : 8;     // Module
            uint64_t c : 8;     // Crate
            uint64_t event : 8; // Event Type
        } triggerId;

        struct {
            // ASSUMING SELF OSCILATION MODE
            // WARNING: The manual specifies that us is in 100ns units,
            //          but I believe that this is wrong.
            uint64_t us : 18; // Module Clock [25ns]
            uint64_t ss : 8;  // Subseconds [1/256s]
            uint64_t s : 30;  // Seconds
            uint64_t event : 8;
        } instrumentTime30;

        struct {
            // ASSUMING SELF OSCILATION MODE
            uint64_t us : 16; // Module Clock [100ns]
            uint64_t ss : 8;  // Subseconds [1/256s]
            uint64_t s : 32;  // Seconds
            uint64_t event : 8;
        } instrumentTime32;
    } event;

    // Convert to host byte order and read into event union
    uint64_t hData;
    memcpy(&hData, buf, 8);
    hData = be64toh(hData);
    memcpy(&event, &hData, 8);

    switch ((TCPEventType)(buf[0])) {
    case TCPEventType::neutronData12: {
        int detector = event.neutronData12.p & 0b111;
        int tof = event.neutronData12.t;
        float pulseR = (float)event.neutronData12.pr;
        float pulseL = (float)event.neutronData12.pl;
        float pulseH = pulseR + pulseL;

        float position;
        if (pulseH != 0) {
            position = pulseL / pulseH;
        } else {
            position = NAN;
        }

        NeutronData neutron = {
            .position = position,
            .detector = detector,
            .triggerOffset = tof,
        };
        return {
            .type = PSDEventType::neutron,
            .neutron = neutron,
        };
    }
    case TCPEventType::neutronData14: {
        int detector = event.neutronData14.p & 0b111;
        int tof = event.neutronData14.t;
        float pulseR = (float)event.neutronData14.pr;
        float pulseL = (float)event.neutronData14.pl;
        float pulseH = pulseR + pulseL;

        float position;
        if (pulseH != 0) {
            position = pulseL / pulseH;
        } else {
            position = NAN;
        }

        NeutronData neutron = {
            .position = position,
            .detector = detector,
            .triggerOffset = tof,
        };
        return {
            .type = PSDEventType::neutron,
            .neutron = neutron,
        };
    }
    case TCPEventType::triggerId:
    case TCPEventType::triggerIdT0Sync: {
        uint8_t c = event.triggerId.c;
        uint8_t m = event.triggerId.m;
        int64_t k = event.triggerId.k;

        TriggerId triggerId = {.crate = c, .module = m, .triggerId = k};
        return {
            .type = PSDEventType::triggerId,
            .triggerId = triggerId,
        };
    }
    case TCPEventType::instrumentTime30: {
        uint32_t s = event.instrumentTime30.s;
        uint32_t ns =
            ((uint32_t)event.instrumentTime30.ss * NS_IN_ONE_256TH_SEC) +
            ((uint32_t)event.instrumentTime30.us * 25);

        epicsTimeStamp ts = {
            .secPastEpoch = s + EPICS_TIME_AT_PSD_EPOCH,
            .nsec = ns,
        };

        return {
            .type = PSDEventType::instrumentTime,
            .instrumentTime = epicsTime(ts),
        };
    }
    case TCPEventType::instrumentTime32: {
        uint32_t s = event.instrumentTime32.s;
        uint32_t ns =
            ((uint32_t)event.instrumentTime32.ss * NS_IN_ONE_256TH_SEC) +
            ((uint32_t)event.instrumentTime32.us * 100);

        epicsTimeStamp ts = {
            .secPastEpoch = s + EPICS_TIME_AT_PSD_EPOCH,
            .nsec = ns,
        };

        return {
            .type = PSDEventType::instrumentTime,
            .instrumentTime = epicsTime(ts),
        };
    }
    default:
        break;
    }

    return {.type = PSDEventType::error};
}

/* UTILITIES */

epicsTime epicsTimeStampFromPSDTime32_t(psdTime32_t src) {
    epicsTimeStamp refTS;
    epicsTimeFromTime_t(&refTS, POSIX_TIME_AT_PSD_EPOCH);
    epicsTime ref = epicsTime(refTS);

    double psdSeconds = (double)(src.s) + ((double)(src.ss) / 256.0);
    return ref + psdSeconds;
}

psdTime32_t epicsTimeToPSDTime32_t(epicsTime src) {
    epicsTimeStamp refTS;
    epicsTimeFromTime_t(&refTS, POSIX_TIME_AT_PSD_EPOCH);
    epicsTime ref = epicsTime(refTS);
    double psdSeconds = (src - ref);

    uint64_t ssFull = std::round(psdSeconds * 256.0);
    uint32_t s = ssFull >> 8;   // Upper 32 bits
    uint8_t ss = ssFull & 0xFF; // Lower 8 bits
    return (psdTime32_t){.s = s, .ss = ss};
}

/* Configuration routine.  Called directly, or from the iocsh function below
 */

extern "C" {

/** EPICS iocsh callable function to call constructor for the
 * testAsynPortDriver class. \param[in] portName The name of the asyn port
 * driver to be created.
 */
int psdPortDriverConfigure(const char *portName, const char *address,
                           const char *tcpPort, const char *udpPort) {
    new psdPortDriver(portName, address, tcpPort, udpPort);
    return (asynSuccess);
}

/* EPICS iocsh shell commands */

static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"address", iocshArgString};
static const iocshArg initArg2 = {"tcpPort", iocshArgString};
static const iocshArg initArg3 = {"udpPort", iocshArgString};
static const iocshArg *const initArgs[] = {&initArg0, &initArg1, &initArg2,
                                           &initArg3};
static const iocshFuncDef initFuncDef = {"psdPortDriverConfigure", 4, initArgs};

static void initCallFunc(const iocshArgBuf *args) {
    psdPortDriverConfigure(args[0].sval, args[1].sval, args[2].sval,
                           args[3].sval);
}

void psdPortDriverRegister(void) { iocshRegister(&initFuncDef, initCallFunc); }

epicsExportRegistrar(psdPortDriverRegister);
}
