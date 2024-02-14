#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include <osiSock.h>
#include <epicsExport.h>
#include <sys/cdefs.h>
#include <sys/param.h>

#include "tEndian.h"
#include "psdPortDriver.h"


static const char *driverName="psdPortDriver";

#define NUM_BINS 320


psdPortDriver::psdPortDriver(const char *portName, const char *address, const char *tcpPort, const char *udpPort)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynFloat64Mask | asynInt32ArrayMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynInt32ArrayMask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
{
    tcpSocket = INVALID_SOCKET;
    udpSocket = INVALID_SOCKET;
    detTCPAddrinfo = NULL;
    detUDPAddrinfo = NULL;

    isConnected = false;
    detAddr = epicsStrDup(address);
    detTCPPort = epicsStrDup(tcpPort);
    detUDPPort = epicsStrDup(udpPort);

    pCounts_ = (epicsInt32 *)calloc(NUM_BINS, sizeof(epicsInt32));

    createParam(P_RunString,                asynParamInt32,         &P_Run);
    createParam(P_HistogramString,          asynParamInt32Array,    &P_Histogram);

    // Set the initial values of some parameters
    setIntegerParam(P_Run,               0);
}

psdPortDriver::~psdPortDriver() {
    this->freeNetworking();

    free(this->detAddr);
    free(this->detTCPPort);
    free(this->detUDPPort);
}

/** Connects driver to device */
asynStatus psdPortDriver::connect(asynUser *pasynUser) {
    int addr;
    asynStatus status = getAddress(pasynUser, &addr);
    if (status != asynSuccess) {
        return status;
    }

    if (isConnected) {
        // Already connected
        pasynManager->exceptionConnect(pasynUser);
        return asynSuccess;
    }

    // Clean up everything
    this->freeNetworking();

    // Create a TCP and UDP socket
    struct addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(detAddr, detTCPPort, &hints, &detTCPAddrinfo) != 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but getaddrinfo failed (TCP)\n",
            driverName, __func__);
        return asynError;
    }

    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(detAddr, detUDPPort, &hints, &detUDPAddrinfo) != 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but getaddrinfo failed (UDP)\n",
            driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    tcpSocket = epicsSocketCreate(detTCPAddrinfo->ai_family, detTCPAddrinfo->ai_socktype, 0);
    if (tcpSocket == INVALID_SOCKET) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but socket creation failed (TCP)\n",
            driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    udpSocket = epicsSocketCreate(detUDPAddrinfo->ai_family, detUDPAddrinfo->ai_socktype, 0);
    if (udpSocket == INVALID_SOCKET) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but socket creation failed (UDP)\n",
            driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    if (::connect(tcpSocket, detTCPAddrinfo->ai_addr, detTCPAddrinfo->ai_addrlen) == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but connecting to socket failed (TCP)\n",
            driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    if (::connect(udpSocket, detUDPAddrinfo->ai_addr, detUDPAddrinfo->ai_addrlen) == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error attempt to connect but connecting to socket failed (UDP)\n",
            driverName, __func__);
        this->freeNetworking();
        return asynError;
    }

    isConnected = true;
    this->setup();

    pasynManager->exceptionConnect(pasynUser);
    return asynSuccess;
}

asynStatus psdPortDriver::disconnect(asynUser *pasynUser) {
    this->teardown();
    this->freeNetworking();
    return asynPortDriver::disconnect(pasynUser);
}

void psdPortDriver::freeNetworking() {
    if (detTCPAddrinfo != NULL) freeaddrinfo(detTCPAddrinfo);
    if (detUDPAddrinfo != NULL) freeaddrinfo(detUDPAddrinfo);
    if (tcpSocket != INVALID_SOCKET) epicsSocketDestroy(tcpSocket);
    if (udpSocket != INVALID_SOCKET) epicsSocketDestroy(udpSocket);

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
    uint8_t type: 4; /* lower 4 bits */
    uint8_t ver:  4; /* upper 4 bits */

    uint8_t flag: 4; /* lower 4 bits */
    uint8_t cmd:  4; /* upper 4 bits */

    uint8_t id;

    uint8_t dataLength;
    beuint32 address;
} __attribute__((packed));

/**
 * Reads/writes data from an to a NEUNET register.
 * Returns the number of bytes written to `readBuf`, or -1 if something failed.
 *
 * \param[in] address Address of the register to write to.
 * \param[in] data Data to write (set to NULL to read).
 * \param[in] dataLength Length of `data` or how many bytes to read.
 * \param[out] readBuf Buffer to copy read bytes to.
 * \param[in] readBufSize Size of `readBuf`.
 */
int psdPortDriver::sendNEUNET(
    uint32_t address, const char* data, uint8_t dataLength,
    char* readBuf, size_t readBufSize
) {
    if (!this->isConnected) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error: not connected\n",
            driverName, __func__);
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
    char *msg = (char*)malloc(msgLength);

    memcpy(msg, &header, sizeof(header));
    memcpy(msg + sizeof(header), data, dataLength);

    // Send Message
    int sentBytes = send(this->udpSocket, msg, msgLength, 0);
    free(msg);

    if (sentBytes == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error: sending failed\n",
            driverName, __func__);
        return -1;
    }

    // Get Response
    char buf[256 + sizeof(UDPMessageHeader)];
    int readBytes = recv(this->udpSocket, buf, sizeof(buf), 0);

    if (readBytes < sizeof(UDPMessageHeader)) {
        return -1;
    }

    auto recvHeader = reinterpret_cast<UDPMessageHeader*>(buf);
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
        memcpy(
            readBuf,
            buf + sizeof(UDPMessageHeader),
            MIN(readBufSize, recvHeader->dataLength)
        );
        return MIN(readBufSize, recvHeader->dataLength);
    }

    return 0;
}

#define NEUNET_ADDR_TIMEMODE   0x18A
#define NEUNET_ADDR_DEVICETIME 0x190
#define NEUNET_ADDR_RW         0x186  /* Why 0x186 and not 0x187 ??? */
#define NEUNET_ADDR_RESOLUTION 0x1B4
#define NEUNET_ADDR_MODE       0x1B5  /* One-Way / Handshake */

int psdPortDriver::setup() {
    // Set time mode to be 32 bit resolution
    const char timeMode32Bit[] = {0x80};
    this->sendNEUNET(NEUNET_ADDR_TIMEMODE, timeMode32Bit, sizeof(timeMode32Bit), NULL, 0);

    // Send current time to device
    // TODO: ...

    // No idea what this does...
    const char evenMemoryReadMode[] = {0x00, 0x00};
    this->sendNEUNET(NEUNET_ADDR_RW, evenMemoryReadMode, sizeof(evenMemoryReadMode), NULL, 0);

    // Set resolution to 14 bit
    const char resolution14Bit[] = {0x8A};
    this->sendNEUNET(NEUNET_ADDR_RESOLUTION, resolution14Bit, sizeof(resolution14Bit), NULL, 0);

    // Switch to oneway mode (disable handshake)
    const char oneWayMode[] = {0x80};
    this->sendNEUNET(NEUNET_ADDR_MODE, oneWayMode, sizeof(oneWayMode), NULL, 0);

    return 0;
}

int psdPortDriver::teardown() {
    // Switch back to handshalke mode
    const char handshakeMode[] = {0x00};
    this->sendNEUNET(NEUNET_ADDR_MODE, handshakeMode, sizeof(handshakeMode), NULL, 0);

    return 0;
}

/** Called when asyn clients call pasynInt32->write().
  * This function sends a signal to the simTask thread if the value of P_Run has changed.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus psdPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_Run) {
        /* If run was set then wake up the simulation task */
        printf("Run will be set set to %d\n", value);
        epicsThreadSleep(0.5);
        printf("Run was set to %d\n", value);

        // SEND A MESSAGE USING THE SOCKET
        if (isConnected) {
            char msg[] = "Run was set!!!\n";
            int len, bytes_sent;
            len = strlen(msg);
            send(tcpSocket, msg, len, 0);
            send(udpSocket, msg, len, 0);
        }


        pCounts_[0] += 1;
    }
    else {
        /* All other parameters just get set in parameter list, no need to
         * act on them here */
    }

    /* Do callbacks so higher layers see any changes */
    status = (asynStatus) callParamCallbacks();
    doCallbacksInt32Array(pCounts_, NUM_BINS, P_Histogram, 0);

    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, name=%s, value=%d",
                  driverName, __func__, status, function, paramName, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%d\n",
              driverName, __func__, function, paramName, value);
    }
    return status;
}

/** Called when asyn clients call pasynFloat64Array->read().
  * Returns the value of the P_Waveform or P_TimeBase arrays.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to read.
  * \param[in] nElements Number of elements to read.
  * \param[out] nIn Number of elements actually read. */
asynStatus psdPortDriver::readInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                         size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    size_t ncopy;
    asynStatus status = asynSuccess;
    epicsTimeStamp timeStamp;

    getTimeStamp(&timeStamp);
    pasynUser->timestamp = timeStamp;

    ncopy = NUM_BINS;
    if (nElements < ncopy) ncopy = nElements;
    if (function == P_Histogram) {
        memcpy(value, pCounts_, ncopy*sizeof(epicsInt32));
        *nIn = ncopy;
    }

    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d",
                  driverName, __func__, status, function);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d\n",
              driverName, __func__, function);
    }
    return status;
}


/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

/** EPICS iocsh callable function to call constructor for the testAsynPortDriver class.
  * \param[in] portName The name of the asyn port driver to be created. */
int psdPortDriverConfigure(
    const char *portName, const char *address,
    const char *tcpPort, const char *udpPort)
{
    new psdPortDriver(portName, address, tcpPort, udpPort);
    return(asynSuccess);
}


/* EPICS iocsh shell commands */

static const iocshArg initArg0 = { "portName", iocshArgString};
static const iocshArg initArg1 = { "address", iocshArgString};
static const iocshArg initArg2 = { "tcpPort", iocshArgString};
static const iocshArg initArg3 = { "udpPort", iocshArgString};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3};
static const iocshFuncDef initFuncDef = {"psdPortDriverConfigure", 4, initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
   psdPortDriverConfigure(args[0].sval, args[1].sval, args[2].sval, args[3].sval);
}

void psdPortDriverRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(psdPortDriverRegister);

}

