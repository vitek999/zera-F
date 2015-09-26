#include <utils/KeyedVector.h>
#include <utils/RefBase.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <utils/String16.h>
#include <utils/threads.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <utils/Log.h>

#include<sys/mount.h>
#include "../../../../kernel/include/mtd/mtd-abi.h"
#include "../../../../kernel/include/mtd/mtd-abi.h"

#include <cutils/atomic.h>
#include <utils/Errors.h>
#include <binder/IServiceManager.h>
#include <utils/String16.h>
#include <binder/IPCThreadState.h>
#include <binder/ProcessState.h>
#include <utils/Vector.h>
#include "libnvram.h"
#include "libfile_op.h"

using namespace android;
enum {
    TRANSACTION_backupBinregionAll = IBinder::FIRST_CALL_TRANSACTION,
};

class INvRAMBackupAgent:public IInterface {
public:
    DECLARE_META_INTERFACE(NvRAMBackupAgent)
    virtual int testBackup()=0;
};

class BpNvRAMBackupAgent: public android::BpInterface<INvRAMBackupAgent>
{
public:
    BpNvRAMBackupAgent(const android::sp<android::IBinder>& impl)
	: android::BpInterface<INvRAMBackupAgent>(impl)
    {

    }
    int testBackup() {return 0;}
};

class BnNvRAMBackupAgent : public BnInterface<INvRAMBackupAgent>
{
public:
    status_t onTransact(uint32_t code,
			const Parcel &data,
			Parcel *reply,
			uint32_t flags);
    
};

class NvRAMBackupAgent : public BnNvRAMBackupAgent
{

public:
    static  void instantiate();
    NvRAMBackupAgent();
    ~NvRAMBackupAgent() {}
    virtual int testBackup();
};


IMPLEMENT_META_INTERFACE(NvRAMBackupAgent, "NvRAMBackupAgent")

