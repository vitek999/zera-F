#ifndef __AFM_MODULE_CKT_SENSOR__
#define __AFM_MODULE_CKT_SENSOR__

#define ERROR "ERROR"
#define RESULT_SIZE 100

class RPCClient;

class ModuleCktSensor {
public:
	ModuleCktSensor();
	virtual ~ModuleCktSensor();
	static int getPSData(RPCClient* msgSender);
	static int getPSOffData(RPCClient* msgSender);
	static int doPSCalibration(RPCClient *msgSender);
	static int clearPSCalibration(RPCClient *msgSender);
};

#endif
