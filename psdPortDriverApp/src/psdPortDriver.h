// Based off testAsynPortDriver.h

#include "asynPortDriver.h"
#include "tEndian.h"
#include <epicsThread.h>
#include <epicsTime.h>
#include <osiSock.h>

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_AcquireString   "PSD_ACQUIRE"   /* asynInt32,    r/w */
#define P_HistogramString "PSD_HISTOGRAM" /* asynInt32Array, r/o */

class psdPortDriver : public asynPortDriver {
public:
    psdPortDriver(const char *portName, const char *address,
                  const char *tcpPort, const char *udpPort);
    ~psdPortDriver();

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                      size_t nElements, size_t *nIn);

    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);

    void readEventLoop();

protected:
    /** Values used for pasynUser->reason, and indexes into the parameter
     * library. */
    int P_Acquire;
    int P_Histogram;

private:
    // Data
    epicsEventId startEventId_;
    epicsEventId stopEventId_;
    epicsInt32 *pCounts_;

    // Networking
    char *detAddr;
    char *detTCPPort;
    char *detUDPPort;

    SOCKET tcpSocket;
    SOCKET udpSocket;
    struct addrinfo *detTCPAddrinfo;
    struct addrinfo *detUDPAddrinfo;

    bool isConnected;

    void freeNetworking();

    int sendNEUNET(uint32_t address, const char *data, uint8_t dataLength,
                   char *readBuf, size_t readBufSize);
    int setup();
    int teardown();

    int readEvent(char *buf);
};

typedef struct {
    float position;
    int detector;
    int triggerOffset; // Measured in 25ns
} NeutronData;

typedef struct {
    int crate;
    int module;
    int64_t triggerId;
} TriggerId;

enum class PSDEventType { neutron, triggerId, instrumentTime, error = -1 };
typedef struct {
    PSDEventType type;
    union {
        NeutronData neutron;
        TriggerId triggerId;
        epicsTime instrumentTime;
    };
} PSDEventData;

PSDEventData parseEventData(char *buf);

/** PSD epoch is 00:00:00 Jan 1, 2008 */
#define POSIX_TIME_AT_PSD_EPOCH 1199145600u
#define EPICS_TIME_AT_PSD_EPOCH                                                \
    (POSIX_TIME_AT_PSD_EPOCH - POSIX_TIME_AT_EPICS_EPOCH)
#define NS_IN_ONE_256TH_SEC 3906250u

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