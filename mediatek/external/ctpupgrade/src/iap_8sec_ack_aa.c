/*  Update Firmware Program
 *	
 *	Programming Procedure:
 *		I2C ISP Address:	0x15
 * 		ISP Mode Command: 	0x54 0x00 0x12 0x34
 *
 *		Step1: Send the I2C Device Write Address(0x15) following the ISP Mode
 * 			   Command data(0x54 0x00 0x12 0x34) to invoke the ISP mode.
 *		Step2: Send the I2C Write Address(0x15) following a data(0x15).
 *		Step3: Extract the 1st page(132 bytes) of data from the .EKT file.
 * 		Step4: Send I2c ISP Write Address(0x15) following the 132 byte data 
 *			   that are extracted from .EKT file, which is the eKTF2000 
 *			   program file.
 * 			   Note: This file contains 251*132 bytes data. Each 132 byte data 
 *				  	 contains 2-byte Address, 128-byte Program data and 2-byte
 *				     Checksum.
 * 		Step5: Send I2C ISP Read Address(0x15) and get ACK data. If ACK is 0x55, 
 *			   it means the checksum fails, go to Step3.
 * 			   Note: if there is checksum error, the programming must be restarted 
 * 				  from the 1st page.
 * 			   If ACK is 0xAA, it means the checksum is ok, extract the next 132 
 * 			   byte of data from the .EKT file and go to Step4.
 * 		Step6: Repeat Step4 and Step5 until all of the 251*132 byte of data are  
 *			   sent to the controller.
 *****************************************************************************/
#include <string.h> 
#include <fcntl.h> 
#include <unistd.h> 
#include <stdlib.h> 
#include <stdio.h> 
#include <linux/i2c.h>
#include <linux/version.h> 
#include <errno.h> 
#include <sys/ioctl.h>
#include <stdint.h>
#include <getopt.h>
#include <sys/types.h>
#include <unistd.h>  
#include <sys/stat.h>

#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54
#define HELLO_PKT			0x55

enum{
	PageSize		= 132,
	
	I2C_Device_Addr = 0x10,
	I2C_ISP_Addr	= 0x15, 
	
	ISP_Mode_0		= 0x54, //54
	ISP_Mode_1		= 0x00, //00
	ISP_Mode_2		= 0x12, //12
	ISP_Mode_3		= 0x34, // 34

	ISP_NEWMode_0		= 0x45,
   	ISP_NEWMode_1		= 0x49,
   	ISP_NEWMode_2		= 0x41,
   	ISP_NEWMode_3		= 0x50,

	ACK_Fail		= 0x55,
	ACK_OK			= 0xAA,

	FW_Ver_0              = 0x53, 
	FW_Ver_1              = 0x00,
	FW_Ver_2              = 0x00,
	FW_Ver_3              = 0x01,	
};

enum{
	E_FD			= 1,
	E_FILE			= 2,
};

enum{
	RK	= 0,
	RESET,
	UNLOCK,
	LOCK,
	INT_PIN,
	TPINFO,
	UPDATE,
};

// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

#define IOCTL_VIAROM	_IOR(ELAN_IOCTLID, 20, int) 
#define IOCTL_VIAROM_CHECKSUM	_IOW(ELAN_IOCTLID, 21, unsigned long)

 
#define ADDRETRY 5
#define PAGERETRY 3
#define retry 2

#define MAX_TRY (3)
//IAP_Addr
char addr;
int iapcmd;
int iapmod = 0;
int byte_mod = 0;

int ic_num, execnum;
uint16_t old_ver, old_id, old_x, old_y, old_bcd;
uint16_t new_ver, new_id, new_x, new_y, new_bcd;
int PageNum = 0;
int Pagesize = 0;
int all_cnt = 0;
int int_low = 1;

static int elan_useage(void)
{
	printf("--------------------------3 parameter--------------------------\n");
	printf("[ELAN] usage: ./APP parameter-1 parameter-2 parameter-3\n");
	printf("------ parameter-1 	xxx.ekt/xxx.ekv\n");
	printf("------ parameter-2 	I2C-7bits adr such as:0x10\n");
	printf("------ parameter-3	2k/3k/rom\n");

	printf("--------------------------1 parameter--------------------------\n");
	printf("[ELAN] usage: ./APP parameter-1\n");
	printf("------ parameter-1 	RK/RESET/LOCK/UNLOCK/TPINFO/INT_PIN/UPDATE\n");

	return 0;
}

static int elan_ktf2k_ts_get_data(int fd, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;

	if (buf == NULL)
		return -EINVAL;

	if (write(fd, cmd, size) != size) {
		printf("send error\n");
		return -1;
	}
#if DEBUG
	else{
		printf("sent %x %x %x %x\n", cmd[0], cmd[1], cmd[2] , cmd[3]);
	}
#endif
	usleep(5000);
		
	if (read(fd, buf, size) != size){
		printf("read error\n");
		return -1;
	}
#if DEBUG
	else{
		printf("recv %x %x %x %x\n", buf[0], buf[1], buf[2] , buf[3]);
	}
#endif
	return 0;
}

int tp_get_info(int iFd, uint16_t *x, uint16_t *y, uint16_t *ver, uint16_t *id, uint16_t *bcd)
{
	int major, minor;
	int rc;
	uint8_t cmd[] 		= {0x53, 0x00, 0x00, 0x01};/*Get Firmware Version*/
	
	uint8_t cmd_x[] 		= {0x53, 0x60, 0x00, 0x00};/*Get x resolution*/
	uint8_t cmd_y[] 		= {0x53, 0x63, 0x00, 0x00};/*Get y resolution*/
	
    uint8_t cmd_bc[] 		= {0x53, 0x10, 0x00, 0x01};/*Get BootCode Version*/
	
	uint8_t cmd_id[] 		= {0x53, 0xf0, 0x00, 0x01};/*Get firmware ID*/
	uint8_t buf_recv[4] 	= {0};
	
// Firmware version
	rc = elan_ktf2k_ts_get_data(iFd, cmd, buf_recv, 4);
	if (rc < 0)
		return -1;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	*ver = major << 8 | minor;
	
// Firmware ID
	rc = elan_ktf2k_ts_get_data(iFd, cmd_id, buf_recv, 4);
	if (rc < 0)
		return -1;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	*id = major << 8 | minor;
	
// X Resolution
	rc = elan_ktf2k_ts_get_data(iFd, cmd_x, buf_recv, 4);
	if (rc < 0)
		return -1;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	*x =minor;
	
	
// Y Resolution	
	rc = elan_ktf2k_ts_get_data(iFd, cmd_y, buf_recv, 4);
	if (rc < 0)
		return -1;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	*y =minor;
	
// BCD version
	rc = elan_ktf2k_ts_get_data(iFd, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return -1;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	*bcd = major << 8 | minor;

	return 0;
}

/***************************************************
 * Main :
 *  Parameter0: execute file name
 *  Parameter1: .EKT file name
 *  Parameter2: update procedure (for test)
 ***************************************************/
int main (int argc, char **argv) 
{ 
   	int iFd, iFile;
	char szOption[4] = {0};
	int iLen = 0;
	int res = 0, s;
	int i;
   	char data;

	printf("This IAP tools is design for elan 1 chip sloution\n");

	iFd = open("/dev/elan-iap",O_RDWR);
	if (iFd < 0) {
		printf("[ELAN] ERROR: Can't open I2C device: /dev/elan-iap\r\n");
		return -1;
	}
	else 
		printf("[ELAN] Open /dev/elan-iap successfully.\r\n");
	
	if (argc == 4) {
		printf("[ELAN] argc num = %d\n", argc);

		addr = (argv[2][2]-'0')*16+(argv[2][3]-'0');
		if(addr != 0x10 && addr != 0x14 && addr != 0x15){
			printf("[ELAN] I2C 7 BITS addr error\n");
			res=-1;
			goto ERR_FILE;
		}
		printf("[ELAN] 7bits i2c addr 0x%X\n", addr);
		
		if(strncmp(argv[3], "2k", 2) == 0){//for 2xxx系列IC
			printf("iap 2k cmd\n");
			iapcmd = 2;
			if(strstr(argv[1], ".ekt") == NULL){
				printf("file error, must need ekt file\n");
				res=-1;
				goto ERR_FILE;
			}
		}
		else if(strncmp(argv[3], "3k", 2) == 0){//for 3xxx系列IC
			printf("iap 3k cmd\n");
			iapcmd = 3;
			if(strstr(argv[1], ".ekt") == NULL){
				printf("file error, must need ekt file\n");
				res=-1;
				goto ERR_FILE;
			}
		}
		else if(strncmp(argv[3], "rom", 3) == 0){//for rom ic
			printf("iap 3k cmd\n");
			iapcmd = 1;
			if(strstr(argv[1], ".ekv") == NULL){
				printf("file error, must need ekv file\n");
				res=-1;
				goto ERR_FILE;
			}
		}
		else{//undefine 
			elan_useage();
			res=-1;
			goto ERR_FILE;
		}
	}
	else if(argc == 2){
		
		res = ioctl(iFd, IOCTL_IAP_MODE_LOCK, data);
		if(strncmp(argv[1], "RK", 2) == 0){
			uint8_t cmd[] = {0x54, 0x29, 0x00, 0x01};
			printf("send cmd: %s\n", argv[1]);
			printf("[elan] dump cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);

			if(write(iFd, cmd, 4) != 4)
				printf("send cmd: %s failed\n", argv[1]);
			else
				printf("send cmd: %s ok\n", argv[1]);
		}
		else if(strncmp(argv[1], "RESET", 6) == 0){
			printf("send cmd: %s\n", argv[1]);
			res = ioctl(iFd, IOCTL_RESET, data);
		}
		else if(strncmp(argv[1], "LOCK", 4) == 0){
			printf("send cmd: %s\n", argv[1]);
			res = ioctl(iFd, IOCTL_IAP_MODE_LOCK, data);
		}
		else if(strncmp(argv[1], "TPINFO", 6) == 0){
			printf("send cmd: %s\n", argv[1]);
			tp_get_info(iFd, &new_x, &new_y, &new_ver, &new_id, &new_bcd);
			
			printf("==========================================================\n");
			printf("[ELAN] New F/W version is 0x%4.4x\n",new_ver);
			printf("[ELAN] New F/W ID is 0x%4.4x\n",new_id);
			printf("[ELAN] New F/W BCD is 0x%4.4x\n",new_bcd);
			printf("[ELAN] New resolution %d,%d\n",new_x,new_y);
			printf("==========================================================\n");
		}
		else if(strncmp(argv[1], "INT_PIN", 7) == 0){
			printf("send cmd: %s\n", argv[1]);
			res = ioctl(iFd, IOCTL_I2C_INT, data);
			printf("--------------int pin %d--------------\n", data);
		}
		else if(strncmp(argv[1], "UPDATE", 7) == 0){
			printf("send cmd: %s\n", argv[1]);
			res = ioctl(iFd, IOCTL_FW_UPDATE, data);
		}
		else{
			elan_useage();
		}
		
		res = ioctl(iFd, IOCTL_IAP_MODE_UNLOCK, data);
		goto ERR_FILE;
	}
	else{
		elan_useage();
		goto ERR_FILE;
	}
	
	iFile = open(argv[1], O_RDONLY);
	if (iFile < 0) {
		printf("[ELAN] ERROR: Can't open source file \"%s\"!\r\n", argv[1]);
		res=-1;
		goto ERR_FILE;
	}
	else
		printf("[ELAN] Open source file \"%s\" successfully.\r\n", argv[1]);

	res = ioctl(iFd, IOCTL_I2C_SLAVE, addr);

	if(res)
	{
		res=-1;
		goto ERR_FILE;
	}
	
	Pagesize = lseek(iFile, 0, SEEK_END);
	PageNum = Pagesize/132;
	
	printf("------------------------------------------------\n");
	printf("+++++++++++++++write %d pages mod+++++++++++++++\n", PageNum);
	printf("------------------------------------------------\n");
	lseek(iFile, 0, SEEK_SET);
	
	res = ioctl(iFd, IOCTL_IAP_MODE_LOCK, data);
	if(res)
	{
		res=-1;
		goto ERR_FD;
	}
	ic_num = 1;
	execnum = 1;

	for(i=0;i<MAX_TRY;i++)
	{
		res=ExecAllSteps(iFile, iFd);
		if(res)
		{
			printf("error TPD_UPGRADE_CKT %d %d\n", i, res);
		}
		else
		{
			printf("done\n");
			break;
		}
	}

  	
ERR_FD:
	ioctl(iFd, IOCTL_IAP_MODE_UNLOCK, data);
	
	ioctl(iFd, IOCTL_RESET, data);
		
	close(iFile);
ERR_FILE:
	close(iFd);

	return 0;
} 

/***************************************************
 * Step 1:
 *  Send I2C write address(0x15) following the ISP Mode 
 *  Command data(0x54 0x00 0x12 0x34)
 ***************************************************/
int EnterISPMode(int file, int fd, int ic)
{ 
	char buff[4] = {0};
	int len = 0;
	int res = 0;
	printf("[ELAN] Enter Step1: EnterISPMode\r\n");
 
	
	if(ic == 1){
		buff[0] = 0x22;
		buff[1] = 0x22;
		buff[2] = 0x22;
		buff[3] = 0x22;
	}
	else if(ic == 2){
		buff[0] = ISP_Mode_0;
		buff[1] = ISP_Mode_1;
		buff[2] = ISP_Mode_2;
		buff[3] = ISP_Mode_3;
	}
	else if(ic == 3){
		buff[0] = ISP_NEWMode_0;
		buff[1] = ISP_NEWMode_1;
		buff[2] = ISP_NEWMode_2;
		buff[3] = ISP_NEWMode_3;
	}
	else{
		buff[0] = 0x33;
		buff[1] = 0x33;
		buff[2] = 0x33;
		buff[3] = 0x33;
	}	
	printf("[ELAN] buff=0x%2x, 0x%2x, 0x%2x, 0x%2x, sizeof(buff)=%d\r\n", buff[0], buff[1], buff[2], buff[3], sizeof(buff));

	len = write(fd, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		printf("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return E_FD;
	}
	else
		printf("[ELAN] EnterISPMode successfully!\r\n");

	return 0;
}

/***************************************************
 * Step 2:
 *  Send the I2C write address(0x15) following a data(0x15) 
 ***************************************************/
int SendI2CWriteAddress(int file, int fd, char data)
{
	char buff[1]={0};
	int len = 0;
	int res = 0;
	printf("[ELAN] Enter Step2: SendI2CWriteAddress\r\n");

	
	buff[0]=data; 
  	printf("[ELAN] buff=0x%2x,sizeof(buff)=%d\r\n", buff[0], sizeof(buff));

	len = write(fd, buff, sizeof(buff));
	
	if (len != sizeof(buff)) {
		printf("[ELAN] ERROR: SendI2CWriteAddress fail! len=%d\r\n", len);
		return E_FD;
	}
	else
		printf("[ELAN] SendI2CWriteAddress successfully!\r\n");

	return 0;
}

int checkiapmod(int fd)
{
	char buff[4]={0};
	int len = 0;
	int res = 0;
	printf("[ELAN] : checkiapmod\r\n");
	
	len = read(fd, buff, sizeof(buff));
	
	if (len != sizeof(buff)) {
		printf("[ELAN] ERROR: checkiapmod fail! len=%d\r\n", len);
		return E_FD;
	}
	else{
		printf("elan check mod %02x %02x %02x %02x\n", buff[0], buff[1], buff[2], buff[3]);
	}
	
	if(buff[0]==0x55 && buff[1]==0xaa && buff[2]==0x33 && buff[3]==0xcc){
		printf("already into isp mod\n");
		return 0;
	}
	else
		return -1;
}

/***************************************************
 * Step 3:
 *  Extract the 1st page(132 byte) of data from the .EKT file. 
 ***************************************************/
int ExtractPage(int file, int fd, char * szPage, int byte)
{
	int len = 0;
	//printf("[ELAN] Enter Step3: ExtractPage\r\n");

	len = read(file, szPage, byte);
	if (len != byte) {
		printf("[ELAN] ERROR: Extract page error, read 50 times error. len=%d\r\n", len);
		return E_FD;
	}
	//else
	//	printf("[ELAN] read page successfully!\r\n");

	return 0;
}

/***************************************************
 * Step 4:
 *  Send the I2C write address following the 132 byte 
 *  that are extracted form .EKT file.
 ***************************************************/
int WritePage(int file, int fd, char * szPage, int byte)
{
	int len = 0;
	int res = 0;
	int i = 0;
	
#if 1
	for(i = 0; i < 16; i++){
		len = write(fd, szPage+(i*8), 8);
		if (len != 8) {
			printf("[ELAN] ERROR: Write page error len=%d\r\n", len);
			return E_FD;
		}
	}
	len = write(fd, szPage+(i*8), 4);
	if (len != 4) {
		printf("[ELAN] ERROR: Write page error len=%d\r\n", len);
		return E_FD;
	}
#else
	len = write(fd, szPage, byte);
	if (len != byte) {
		printf("[ELAN] ERROR: Write page error len=%d\r\n", len);
		return E_FD;
	}
#endif
	
	return 0;
}

/***************************************************
 * Step 5:
 *  Send I2C ISP read address(0x15) and get ACK data
 ***************************************************/
int GetAckData(int file, int fd)
{
	int len = 0;
	int res = 0;
	char buff[2] = {0};
	int cnt = 0;
	int intlow = 1;
	
	len = read(fd, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		printf("[ELAN] ERROR: Get ACK data error, read IC no ack. len=%d\r\n", len);
		return E_FD;
	}
	
	if (buff[0] != 0xaa){
		printf("[ELAN] ack error buff[0]=0x%x  buff[1]=0x%x\r\n", buff[0], buff[1]);
		
		return buff[0];
	}
	else 
		return ACK_OK;
	
	return 0;
}

/***************************************************
 * Step 65:
 *  Send I2C ISP read address(0x15) and get Hello data
 ***************************************************/
int GetHelloData(int file, int fd)
{
	int len = 0;
	int res = 0;
	char data;
	int i;
	char buff[4] = {0};
	printf("[ELAN] Enter Step5: GetHelloData~~~~\r\n");

	res = ioctl(fd, IOCTL_RESET, data);

	poll(fd);
	
	len = read(fd, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		printf("[ELAN] ERROR: read hello data error,len=%d\r\n", len);
		return E_FD;
	}
	else{
		printf("[ELAN] read Hello data successfully! buff[0]=0x%x  buff[1]=0x%x buff[2]=0x%x  buff[3]=0x%x \r\n", buff[0], buff[1], buff[2], buff[3]);
	}
	//add message for "55" - "AA"
	if (buff[0] != 0x55 && buff[1] != 0x55 && buff[2] != 0x55 && buff[3] != 0x55){
		return E_FD;
	}

	return 0;
}

/*check int pin if low*/
int poll(int fd)
{
	int int_poll = 0;
	int res;	
	do{
		usleep(20000);
		res = ioctl(fd, IOCTL_I2C_INT, &int_low);
		printf("elan waite int_low %d ?= 0\n", int_low);					
		if(int_poll == 50){
			break;
		}
		int_poll++;
	}while(int_low != 0);
	return 0;
}

/***************************************************
 *  Execute all  updated procedure 
 ***************************************************/
int ExecAllSteps(int file, int fd) 
{
	int res = 0;
	int iPage = 0, ackcnt = 0;
	int i = 0, sndisp=0, sndid=0, j;
	char szBuff[132]={0};
	
	char hello[4]={0};
	char sleep[4]={0x54, 0x50, 0x00, 0x01}; 
	char hello_recover[4] = {0};
	char check_sum[4] = {0};
	
	char data;
	int byte_count;
	char recover = 0x55;

	struct timeval struc_start, struc_end;  long dif_time = 0;
	
	tp_get_info(fd, &old_x, &old_y, &old_ver, &old_id, &old_bcd);
	
	//get start time
  	gettimeofday(&struc_start, NULL);
  	long start = ((long)struc_start.tv_sec)*1000 +  (long)struc_start.tv_usec/1000;
	
	printf("[ELAN] ExecAllStep: execnum=%d ic_num=%d\r\n", execnum, ic_num);
	
	for(j=0;j<execnum;j++){
      	for (i=ic_num; i>0; i--) { 
			   			
			data= addr;
			printf("[ELAN] data=0x%x \r\n", data);

			if(iapcmd == 1){
				printf("---------start change driver checksum---------\n");
				
				lseek(file, 2238, SEEK_SET);
				read(file, check_sum, 4);
				lseek(file, 0, SEEK_SET);
				
				printf("ekv checksum %x %x %x %x\n", check_sum[0], check_sum[1], check_sum[2], check_sum[3]);
				res = ioctl(fd, IOCTL_VIAROM_CHECKSUM, check_sum);
				printf("---------end change driver checksum---------\n");
				
			}
reset:

			if(iapcmd == 2){
				char hello[8] = {0};
				res = ioctl(fd, IOCTL_RESET, data);
				usleep(100000);
							
				poll(fd);
				
				res = read(fd, hello, 8);
				if(res != 8){
					printf("elan recv hello error\n");
					return E_FD;
				}
				recover = hello[2];
				
				if(hello[2] != 0x80 && hello[2] != 0x55){
					printf("elan unknow error\n");
					return E_FD;			
				}

				if(recover == 0x55){
					printf("+++++++normal mod++++++++\n");
					res = EnterISPMode(file, fd, iapcmd);	 			/*Step 1 enter ISP mode*/
					checkiapmod(fd);
				}
				else{
					printf("+++++++recover mod++++++++\n");
				}
			}

			if(iapcmd == 3){
				res = ioctl(fd, IOCTL_RESET, data);
				res = EnterISPMode(file, fd, iapcmd);	 			/*Step 1 enter ISP mode*/
				usleep(5000);
				checkiapmod(fd);
			}
			usleep(50000);
		   	res = SendI2CWriteAddress(file, fd, data); 				/*Step 3 sned slave address following a data()0x15*/
			if(res != 0){
				printf("[ELAN] SendI2CWriteAddress fail................\n");
				return res;
			}
	    		
			usleep(50000);	
			lseek(file, 0, SEEK_SET);
				
			for(iPage = 1; iPage <= PageNum; iPage++) {
				
				printf("page %d\n",iPage);	
				res = ExtractPage(file, fd, szBuff, 132); /*Step 4 extract the 1st page of data from the .EKT file*/
				if (0 != res) {
					printf("[ELAN] ERROR: ExtractPage \"%d\" fail!\r\n", iPage);
					return res;
				}
PAGE_REWRITE:
				res = WritePage(file, fd, szBuff, 132);
				
				if(iapcmd != 1){		
					if(iPage == 1 || iPage == 249)
						usleep(200000);
					else
			 			usleep(30000*(ackcnt+1));
				}
				res = GetAckData(file, fd);	/*Step 5 get ACK data*/
				
			 	if (ACK_OK != res) {
					printf("[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
					ackcnt = ackcnt + 1;
					if (ackcnt == PAGERETRY){
						printf("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n",  data, iPage, PAGERETRY);
						usleep(100000);
						all_cnt = all_cnt + 1;
						if(all_cnt < retry){
							ackcnt=0;
							goto reset;
						}
						else{
							return E_FD;
						}
						
					 }
					else{
						printf("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, ackcnt);
						goto PAGE_REWRITE;
					}
				} 
				else {
				         ackcnt=0;
			 	}
				
				printf("[ELAN] IC addr-->0x%02x page-->%dth write successfully!!!\n\n", data, iPage);
			}
           	}              
   	}

	res = ioctl(fd, IOCTL_RESET, data);	 
	res = GetHelloData(file, fd);	/*Step 6 get hello*/

	//get end time
	gettimeofday(&struc_end, NULL);
	long end = ((long)struc_end.tv_sec)*1000 + (long)struc_end.tv_usec/1000;

	//calculate time slot
	dif_time = end - start;

	printf("\n\n+++++++++++++++++++++++cost %ld s--%ld ms++++++++++++++++++++++++++\n\n", dif_time/1000, dif_time%1000);

	if(iapcmd == 1){
		res = EnterISPMode(file, fd, 0);
		usleep(50000);
	}


	tp_get_info(fd, &new_x, &new_y, &new_ver, &new_id, &new_bcd);
	
	printf("==========================================================\n");
  	printf("[ELAN] Previous F/W version is 0x%4.4x\n",old_ver);
  	printf("[ELAN] Previous F/W ID is 0x%4.4x\n",old_id);
	printf("[ELAN] Previous F/W BCD is 0x%4.4x\n",old_bcd);
  	printf("[ELAN] Previous resolution %d,%d\n\n",old_x,old_y);
	printf("[ELAN] New F/W version is 0x%4.4x\n",new_ver);
	printf("[ELAN] New F/W ID is 0x%4.4x\n",new_id);
	printf("[ELAN] New F/W BCD is 0x%4.4x\n",new_bcd);
	printf("[ELAN] New resolution %d,%d\n",new_x,new_y);
	printf("==========================================================\n");
	
    return 0;
}
