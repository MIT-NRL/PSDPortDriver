// Based off testAsynPortDriver.h

#include "asynPortDriver.h"
#include "epicsTypes.h"
#include "include/tEndian.h"
#include <array>
#include <epicsThread.h>
#include <epicsTime.h>
#include <osiSock.h>

#define PSD_MAX_BINS      512
#define PSD_NUM_DETECTORS 8

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_AcquireString         "ACQUIRE"           /* asynInt32,      r/w */
#define P_AcquireTimeString     "ACQ_TIME"          /* asynFloat64,    r/w */
#define P_AcquireTimeRemainingString "ACQ_TIME_REMAINING" /* asynFloat64, r/o */
#define P_NumBinsString         "NUM_BINS"          /* asynInt32,      r/w */
#define P_SoftLLDString         "SOFT_LLD"          /* asynInt32,      r/w */
#define P_SoftHLDString         "SOFT_HLD"          /* asynInt32,      r/w */
#define P_HardLLDString         "Hard_LLD"          /* asynInt32,      r/w */
#define P_CountsString          "COUNTS"            /* asynInt32Array, r/o */
#define P_TotalCountsString     "TOTAL_COUNTS"      /* asynInt64Array, r/o */
#define P_LiveCountsString      "LIVE_COUNTS"       /* asynInt32Array, r/o */
#define P_LiveTotalCountsString "LIVE_TOTAL_COUNTS" /* asynInt64Array, r/o */
#define P_ConnectString         "CONNECT"           /* asynInt32,      r/w */
#define P_ModeString            "MODE"              /* asynBool,       r/w */
#define P_RegDumpRawString      "REG_DUMP_RAW"      /* asynOctet,      r/o */
#define P_RegDumpLine0String    "REG_DUMP_LINE0"    /* asynOctet,      r/o */
#define P_RegDumpLine1String    "REG_DUMP_LINE1"    /* asynOctet,      r/o */
#define P_RegDumpLine2String    "REG_DUMP_LINE2"    /* asynOctet,      r/o */
#define P_RegDumpLine3String    "REG_DUMP_LINE3"    /* asynOctet,      r/o */
#define P_RegDumpLine4String    "REG_DUMP_LINE4"    /* asynOctet,      r/o */
#define P_RegDumpLine5String    "REG_DUMP_LINE5"    /* asynOctet,      r/o */
#define P_RegDumpLine6String    "REG_DUMP_LINE6"    /* asynOctet,      r/o */

class psdPortDriver : public asynPortDriver {
public:
    psdPortDriver(const char *portName, const char *address,
                  const char *tcpPort, const char *udpPort);
    ~psdPortDriver();

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                      size_t nElements, size_t *nIn);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value,
                                 size_t maxChars, size_t *nActual,
                                 int *eomReason);

    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);

    void readEventLoop();
    void registerDumpLoop();

protected:
    /** Values used for pasynUser->reason, and indexes into the parameter
     * library. */
    int P_Acquire;
    int P_AcquireTime;
    int P_AcquireTimeRemaining;
    int P_NumBins;
    int P_SoftLLD;
    int P_SoftHLD;
    int P_HardLLD;
    int P_Counts;
    int P_TotalCounts;
    int P_LiveCounts;
    int P_LiveTotalCounts;
    int P_Connect;
    int P_Mode;
    int P_RegDumpRaw;
    int P_RegDumpLine0;
    int P_RegDumpLine1;
    int P_RegDumpLine2;
    int P_RegDumpLine3;
    int P_RegDumpLine4;
    int P_RegDumpLine5;
    int P_RegDumpLine6;

private:
    // Data
    epicsEventId startEventId_;
    epicsEventId stopEventId_;
    std::array<epicsInt32, PSD_NUM_DETECTORS * PSD_MAX_BINS> counts_;
    std::array<epicsInt64, PSD_NUM_DETECTORS> totalCounts_;

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
    int setTransferMode(bool oneWay);
    void flushNEUNET();
    int setHardLLD(int lldValue);
    void refreshRegisterDumpLocked();
    void setRegisterDumpDisconnectedLocked();

    int readEvent(char *buf);
    void realignTCP();
};

typedef struct {
    float position;
    int pulseHeight;
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
