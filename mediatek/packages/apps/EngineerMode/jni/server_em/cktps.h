#ifndef CKTPS_H
#define CKTPS_H


//宏定义
#define ALSPS							0X84
#define ALSPS_SET_PS_MODE					_IOW(ALSPS, 0x01, int)
#define ALSPS_GET_PS_MODE					_IOR(ALSPS, 0x02, int)
#define ALSPS_GET_PS_DATA					_IOR(ALSPS, 0x03, int)
#define ALSPS_GET_PS_RAW_DATA				_IOR(ALSPS, 0x04, int)
#define ALSPS_SET_ALS_MODE					_IOW(ALSPS, 0x05, int)
#define ALSPS_GET_ALS_MODE					_IOR(ALSPS, 0x06, int)
#define ALSPS_GET_ALS_DATA					_IOR(ALSPS, 0x07, int)
#define ALSPS_GET_ALS_RAW_DATA           	_IOR(ALSPS, 0x08, int)
#define ALSPS_SET_PS_CALI           	    _IOR(ALSPS, 0x17, int)
// xiangfei.peng 20140513 because _IOR(ALSPS, 0x09, int) is already defined for ALSPS_GET_PS_TEST_RESULT
//数据结构定义

//函数定义
int get_ps_data(int *data);
int read_offdata(int *data);
int do_ps_calibration (int *pdata);
int clear_ps_calibration (int *pdata);
//全局变量声明

#endif /* CKTPS_H */




 


