// Based off testAsynPortDriver.h

#include <epicsTime.h>
#include <osiSock.h>
#include "tEndian.h"
#include "asynPortDriver.h"

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_RunString                "PSD_RUN"                  /* asynInt32,    r/w */
#define P_HistogramString          "PSD_HISTOGRAM"            /* asynInt32Array, r/o */

class psdPortDriver: public asynPortDriver {
public:
    psdPortDriver(const char *portName, const char *address, const char *tcpPort, const char *udpPort);
    ~psdPortDriver();

    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                        size_t nElements, size_t *nIn);

protected:
    /** Values used for pasynUser->reason, and indexes into the parameter library. */
    int P_Run;
    int P_Histogram;

private:
    // Data
    epicsInt32 *pCounts_;

    /** The TCP socket */
    SOCKET tcpSocket;
    /** The UDP socket */
    SOCKET udpSocket;
    /** Detector TCP addrinfo */
    struct addrinfo *detTCPAddrinfo;
    /** Detector UDP addrinfo */
    struct addrinfo *detUDPAddrinfo;

    bool isConnected;

    /** Detector address */
    char *detAddr;
    /** Detector TCP port */
    char *detTCPPort;
    /** Detector UDP port */
    char *detUDPPort;


    void freeNetworking();
    int sendNEUNET(
        uint32_t address, const char* data, uint8_t dataLength,
        char* readBuf, size_t readBufSize);

    int setup();
    int teardown();
};

/** PSD epoch is 00:00:00 Jan 1, 2008 */
#define POSIX_TIME_AT_PSD_EPOCH 1199145600u
#define EPICS_TIME_AT_PSD_EPOCH (POSIX_TIME_AT_PSD_EPOCH - POSIX_TIME_AT_EPICS_EPOCH)

/** In "32bit" mode, 4 bytes are used for the seconds and one for subseconds
  * This struct is stored in network byte order (big endian) */
typedef struct __attribute__((packed)) {
    /** Seconds since PSD Epoch */
    beuint32 s;
    /** Subseconds (unit 2^-8 sec) */
    uint8_t ss;
} psdTime32_t;

/** Convert a 32 bit timestamp from the PSD to an epics time */
epicsTime epicsTimeFromPSDTime32_t(psdTime32_t src);

/** Convert an epics time to a 32 bit timestamp for the PSD */
psdTime32_t epicsTimeToPSDTime32_t(epicsTime src);