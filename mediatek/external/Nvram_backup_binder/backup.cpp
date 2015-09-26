#include "backup.h"
#include "libnvram.h"
#include "libfile_op.h"
#include <cutils/xlog.h>
extern int iFilePRODUCT_INFOLID;

void NvRAMBackupAgent::instantiate() {

int ret = (int)defaultServiceManager()->addService(descriptor, new NvRAMBackupAgent());

   if(ret != 0){
       XLOGE("Haman, nvram backup serviceManager not ready");
       exit(1);
   }
   XLOGE("Haman, nvram backup serviceManager work well");

}

NvRAMBackupAgent::NvRAMBackupAgent() {
    NVRAM_LOG("NvRAMBackupAgent created");
}

status_t BnNvRAMBackupAgent::onTransact(uint32_t code,
			       const Parcel &data,
			       Parcel *reply,
			       uint32_t flags) {

    XLOGD("OnTransact   (%u,%u)", code, flags);
        
    switch(code) {
    case TRANSACTION_backupBinregionAll: 
        {
	        XLOGD("backup\n");
	        int ret=0;
	        ret=testBackup ();
	        if (ret==0) {
		        NVRAM_LOG("backup Error\n");
	        } else {
		        NVRAM_LOG("backup done!\n");
		    }
	        return NO_ERROR;
        } 
        break;
    
    default:
	    return BBinder::onTransact(code, data, reply, flags);
    }

    return NO_ERROR;
}

int NvRAMBackupAgent::testBackup() {

    if(FileOp_BackupToBinRegion_All())
        return 1;
    else 
        return 0;
}

int main(int argc, char *argv[])
{
    NvRAMBackupAgent::instantiate();
    ProcessState::self()->startThreadPool();
    NVRAM_LOG("NvRAMBackupAgent Service is now ready");
    IPCThreadState::self()->joinThreadPool();
    return(0);
}

