#define LOG_TAG "EMSENSOR"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cutils/xlog.h>
#include "ModuleCktSensor.h"
#include "RPCClient.h"
extern "C"{
#include "libhwm.h"
#include "cktps.h"
}

int ModuleCktSensor::getPSData(RPCClient* msgSender) {
	int paraNum = msgSender->ReadInt();
	int tolerance = 0;
	if (paraNum != 0) {
		msgSender->PostMsg((char*) ERROR);
		return -1;
	}
	int data=0;
// 苏 勇 2013年09月23日 15:17:55	XLOGD("Enter getPSData()\n");
	int ret = get_ps_data(&data);
// 苏 勇 2013年09月23日 15:17:58	XLOGD("getPSData() returned %d, %d\n", ret, data);

	char result[RESULT_SIZE] = { 0 };
	sprintf(result, "%d", ret?0:1);
	msgSender->PostMsg(result);
	sprintf(result, "%d", data);
	msgSender->PostMsg(result);
	return 0;
}

int ModuleCktSensor::getPSOffData(RPCClient* msgSender) {
	int paraNum = msgSender->ReadInt();
	int tolerance = 0;
	if (paraNum != 0) {
		msgSender->PostMsg((char*) ERROR);
		return -1;
	}
	int data=0;
// 苏 勇 2013年09月23日 15:18:01	XLOGD("Enter getPSOffData()\n");
	int ret = read_offdata(&data);
// 苏 勇 2013年09月23日 15:18:03	XLOGD("getPSOffData() returned %d, %d\n", ret, data);

	char result[RESULT_SIZE] = { 0 };
	sprintf(result, "%d", ret?0:1);
	msgSender->PostMsg(result);
	sprintf(result, "%d", data);
	msgSender->PostMsg(result);
	return 0;
}

int ModuleCktSensor::doPSCalibration(RPCClient * msgSender)
{
	int paraNum = msgSender->ReadInt();
	int tolerance = 0;
	if (paraNum != 0) {
		msgSender->PostMsg((char*) ERROR);
		return -1;
	}
	int data=0;
// 苏 勇 2013年10月08日 11:50:05	XLOGD("Enter doPSCalibration()\n");
	int ret = do_ps_calibration(&data);
// 苏 勇 2013年10月08日 11:50:08	XLOGD("doPSCalibration() returned %d, %d\n", ret, data);

	char result[RESULT_SIZE] = { 0 };
	sprintf(result, "%d", ret?0:1);
	msgSender->PostMsg(result);
	sprintf(result, "%d", 0);
	msgSender->PostMsg(result);
	return 0;
}

int ModuleCktSensor::clearPSCalibration(RPCClient * msgSender)
{
	int paraNum = msgSender->ReadInt();
	int tolerance = 0;
	if (paraNum != 0) {
		msgSender->PostMsg((char*) ERROR);
		return -1;
	}
	int data=0;
// 苏 勇 2013年10月08日 11:50:10	XLOGD("Enter clearPSCalibration()\n");
	int ret = clear_ps_calibration(&data);
// 苏 勇 2013年10月08日 11:50:12	XLOGD("clearPSCalibration() returned %d, %d\n", ret, data);

	char result[RESULT_SIZE] = { 0 };
	sprintf(result, "%d", ret?0:1);
	msgSender->PostMsg(result);
	sprintf(result, "%d", 0);
	msgSender->PostMsg(result);
	return 0;
}

ModuleCktSensor::ModuleCktSensor(void) {
}

ModuleCktSensor::~ModuleCktSensor(void) {
}

