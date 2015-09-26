/******************** (C) COPYRIGHT 2012 STMicroelectronics *******************
* File Name      : main.c
*                : AMS - MEMS & Sensor CHINA - Application Team
*                : Yi WU (yi.wu@st.com)
* Version        : Revision: 1.0
* Description    : Motion Sensors 9-axis Fusion Daemon
*******************************************************************************
* History:
* Date          | Modification               | Author
*
* 27/05/2014    | First release V 1.0        | MEMS & Sensor CHINA Application Team
*                                            | Yi WU
*
*******************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR REFERENCE OR
* EDUCATION. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
******************************************************************************/

/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <time.h>

#ifdef MTK_RIL_MD2
#define LOG_TAG "RILDMD2"
#else
#define LOG_TAG "RILD"
#endif

#include <utils/Log.h>
#include <cutils/properties.h>
#include <cutils/sockets.h>
#include <linux/capability.h>
#include <linux/prctl.h>

#include <poll.h>
#include <dirent.h>
#include <sys/select.h>

//commom
#include <math.h>
#include <hardware/sensors.h>
#include <sys/types.h>

#include <linux/ioctl.h>

#include <sys/stat.h>
#include <sys/ioctl.h>

#include <sys/time.h>
#include <signal.h>

#include <linux/sensors_io.h>
#include <linux/hwmsensor.h>
//#include "Lsm330Measure.h"

#include "iNemoEngineAPI.h"

#include <private/android_filesystem_config.h>

//#define DEBUG_ENG

static int Daemon_Debug =  1;

//added by ST WU Yi 2013.11.25.
#define  GYROSCOPE_IOC_GET_OFLAG  _IOR(GYROSCOPE, 0x0a, int)
#define  GYROSCOPE_IOC_GET_GRFLAG  _IOR(GYROSCOPE, 0x0b, int)
#define  GYROSCOPE_IOC_GET_RVFLAG  _IOR(GYROSCOPE, 0x0c, int)
#define  GYROSCOPE_IOC_GET_GYFLAG  _IOR(GYROSCOPE, 0x0d, int)
#define  GYROSCOPE_IOC_GET_LAFLAG  _IOR(GYROSCOPE, 0x0e, int)
#define  GYROSCOPE_IOC_GET_OPEN_STATUS  _IOR(GYROSCOPE, 0x0f, int)
#define  GYROSCOPE_IOC_GET_SUSPEND_STATUS  _IOR(GYROSCOPE, 0x018, int)

#define GYROSCOPE_IOCTL_SET_ORIENTATION        	      _IOW(GYROSCOPE, 0x10, int)
#define GYROSCOPE_IOCTL_READ_ORIENTATION_DATA		  _IOR(GYROSCOPE, 0x11, int)
#define GYROSCOPE_IOCTL_SET_ROTATION_VECTOR  	      _IOW(GYROSCOPE, 0x12, int)
#define GYROSCOPE_IOCTL_READ_ROTATION_VECTOR_DATA	  _IOR(GYROSCOPE, 0x13, int)
#define GYROSCOPE_IOCTL_SET_GRAVITY  	              _IOW(GYROSCOPE, 0x14, int)
#define GYROSCOPE_IOCTL_READ_GRAVITY_DATA	          _IOR(GYROSCOPE, 0x15, int)
#define GYROSCOPE_IOCTL_SET_LINEAR_ACC  	          _IOW(GYROSCOPE, 0x16, int)
#define GYROSCOPE_IOCTL_READ_LINEAR_ACC_DATA	      _IOR(GYROSCOPE, 0x17,int)

#if 1
#define LOGD(fmt,args...)   \
	do { \
	     if(Daemon_Debug)  ALOGD(fmt,##args);\
		}while(0);

#else
#define LOGD(fmt,args...)   ALOGD(fmt,##args)
#endif

#if 1
#define LOGD(fmt,args...)   \
	do { \
	     if(Daemon_Debug)  ALOGD(fmt,##args);\
		}while(0);

#else
#define LOGD(fmt,args...)   ALOGD(fmt,##args)
#endif

#define FN 10
#define QTH 1500
#define DELTA_G 0.15
#define FIRST_WRONG_DATA_NUM 20

iNemoSensorsData sdata;

int accfp = 0;
int magfp = 0;
int gyrfp = 0;
int gy_flag = 0;
int or_flag = 0;
int rv_flag = 0;
int gr_flag = 0;
int la_flag = 0;
int PNI_9X_opened = 0;
static float accx_first_data[FIRST_WRONG_DATA_NUM] ={0.0};
static float accy_first_data[FIRST_WRONG_DATA_NUM] ={0.0};
static float accz_first_data[FIRST_WRONG_DATA_NUM] ={0.0};

long long getTimestamp()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    long long ul = (t.tv_sec)*1000000000LL + t.tv_nsec;
    return ul;
}

static int control_read_virtual_sensors_state(int fd)
{
    if (fd < 0)
    {
        return 0;
    }

    int sensors = 0;
    int suspend_gy =0;

    int sensordata[2]={0};
    // read the actual value of all sensors
    if(!ioctl(fd, GYROSCOPE_IOC_GET_GRFLAG, &gr_flag)
       &&!ioctl(fd, GYROSCOPE_IOC_GET_OFLAG, &or_flag)
       &&!ioctl(fd, GYROSCOPE_IOC_GET_RVFLAG, &rv_flag)
       &&!ioctl(fd, GYROSCOPE_IOC_GET_LAFLAG, &la_flag)
       &&!ioctl(fd, GYROSCOPE_IOC_GET_GYFLAG, sensordata))
        {
        gy_flag = sensordata[0];
#ifdef DEBUG_ENG
        LOGD(" ST ALG  st alg  gy_flag %d\n", gy_flag);
#endif
        //if(!or_flag && !rv_flag && !gr_flag && !la_flag && !gy_flag)
        if( !rv_flag && !gr_flag && !la_flag && !gy_flag)
        {
            sensors = 0;
        }
        else
        {
            sensors = 1;
        }
    }
#ifdef DEBUG_ENG
    LOGD(" ST ALG  st alg===  control_read_virtual_sensors_state  %d\n", sensors);
#endif


    suspend_gy	 =  sensordata [1];

#ifdef DEBUG_ENG
    LOGD(" ST ALG  st alg===  control_read_virtual_sensors_state  suspend_gy =  %d\n", suspend_gy);
#endif
    if (suspend_gy ==1)
    {
        return 0;
    }
    else
    {
        return sensors;
    }
}

int main()
{
/*
    int chipinfo_fd = open("/sys/bus/platform/drivers/gyroscope/chipinfo", O_RDWR);
    char flag[3];
    read(chipinfo_fd, flag, 3);
    close(chipinfo_fd);
    printf("flag : %c \n", flag[0]);

    if( flag[0] == 'L' )
    {
        printf("lsm330fd run  for chipinfo  match. \n");
    }
    else
    {
        printf("lsm330fd  exit  for chipinfo not match. \n");
        exit(0);
    }
*/
    int len = sizeof(long long);
#ifdef DEBUG_ENG
    LOGD(" ST ALG  st algRILD started len = %d", len);
#endif
    int iStatus;
    int res;
    int sensor_stat = 0;
    int x,y,z;
    int ax, ay, az;
    int mx, my, mz;
    float gx, gy, gz;
    int delay;
    static long long prev_time = 0;
    char buff_data[32];
    int posturebuf[4];
    int posturebufgyro[4];
    float data[4];
    int first_10_data = 0;
    static float sensor_data[9];
    sensor_data[0] = 0;
    sensor_data[1] = 0;
    sensor_data[2] = 9.8;
    sensor_data[3] = 0;
    sensor_data[4] = 30;
    sensor_data[5] = 25;
    sensor_data[6] = 0;
    sensor_data[7] = 0;
    sensor_data[8] = 0;

    static float gyx[FN];
    static float gyy[FN];
    static float gyz[FN];

    static int First_in_lib =1;

    int j = 0;
    for(j=0; j<FN; j++)
    {
        gyx[j]=gyy[j]=gyz[j]=0;
    }

//added by ST WU Yi for control acc & mag
/*
    static int openflag = 0;
    static int delay_ms = 10;
    static int pre_acc_delay_ms = 10;
    static int pre_mag_delay_ms = 10;
    static int pre_acc_open = 0;
    static int pre_mag_open = 0;

    ioctl(accfp, GSENSOR_IOC_GET_ENABLE, &pre_acc_open);
    ioctl(magfp, MSENSOR_IOC_GET_ENABLE, &pre_mag_open);
    ioctl(accfp, GSENSOR_IOC_GET_DELAY, &pre_acc_delay_ms);
    ioctl(magfp, MSENSOR_IOC_GET_DELAY, &pre_mag_delay_ms);
*/

    iNemoDebugInitData debug_init_data_api;
    iNemoInitData init_data_api;
    iNemoDebugOutData debug_out_data;
    float gbias[3]={0.0, 0.0, 0.0};

    mx = my = mz = 0;
    ax = ay = az = 0;
    gx = gy = gz = 0;
    iStatus = 2;

    int stat_curr = 0;
    float t;

    long long cur_t = 0;
    long long cur_t1, cur_t5;
    cur_t1 = cur_t5 = 0;

    accfp = open("/dev/gsensor", O_RDWR);
    if(accfp == -1)
    {
        LOGD(" ST ALG  st algCannot open gsensor!!!\n");
        return -1;
    }

    magfp = open("/dev/msensor", O_RDWR);
    if(magfp == -1)
    {
        LOGD(" ST ALG  st algCannot open msensor!!!\n");
        return -1;
    }

    gyrfp = open("/dev/gyroscope", O_RDWR);
    if(gyrfp == -1)
    {
        close(accfp);
        close(magfp);
        LOGD(" ST ALG  st algCannot open gyroscope!!!\n");
		return -1;
    }
    if(ioctl(gyrfp, GYROSCOPE_IOCTL_CLR_CALI)<0)
    {
        LOGD(" ST ALG GYROSCOPE_IOCTL_CLR_CALI failed\n");
        return -1;
    }

    int gyrofd_a = -1;
    int accfd_a = -1;
    int magfd_a = -1;
    int openflag = 0;
    int pre_acc_open;
    int pre_mag_open;
    int pre_acc_delay_ms;
    int pre_mag_delay_ms;
    int delay_ms;

    init_data_api.LocalEarthMagField = 50.0;
    init_data_api.GbiasLearningMode = 2;
    init_data_api.Gbias_threshold_accel = 7.4399903e-4/7*1.5*1.13; //6.6299865e-4;//525e-6; //3.2699906e-4;//3.4499902e-4;//550e-6;//4.949991e-4;//
    init_data_api.Gbias_threshold_magn  = 7.679987e-4/0.9;//7.1399874e-4;//230e-6;//7.4999884e-4;//7.589986e-4;//1471e-6;//8.819988e-4;//
    init_data_api.Gbias_threshold_gyro  = 9.989983e-4/8*3*1.13;//9.689984e-4;//340e-6;//0.001025998;//0.0011069984;//1491e-6;//0.0011609982;//
    init_data_api.ATime = -1;
    init_data_api.MTime = -1;
    init_data_api.PTime = -1;
    init_data_api.FrTime = -1;
    init_data_api.gbias_file = NULL;

    debug_init_data_api.accel_flag = 0;
    debug_init_data_api.magn_flag = 0;
    debug_init_data_api.gyro_flag = 0;

    debug_init_data_api.accel_flag = 1;
    debug_init_data_api.magn_flag = 1;
    debug_init_data_api.gyro_flag = 1;
    iNemoEngine_API_Initialization(&init_data_api, &debug_init_data_api);

    iNemoEngine_API_enable_9X(true);

    PNI_9X_opened = 1;

	while(1)
    {
        cur_t1 = getTimestamp()/1000;

#ifdef DEBUG_ENG
        LOGD("ST ALG current time1 = %lld", cur_t1);
#endif

	    stat_curr = control_read_virtual_sensors_state(gyrfp);
        //LOGD("ST ALG stat_curr  = %d", stat_curr );
	    if(stat_curr == 0)
	    {
            first_10_data = 0;

            if(PNI_9X_opened ==1)
            {
                iNemoEngine_API_enable_9X(false);
                //iNemoEngine_API_enable_6X(false);
                PNI_9X_opened = 0;
            }

            if(openflag==1)
            {
                //added by ST WU Yi for control acc & mag
                openflag = 0;
                //ioctl(accfp, GSENSOR_IOC_SET_ENABLE, &pre_acc_open);
                //ioctl(magfp, MSENSOR_IOCTL_SENSOR_ENABLE, &pre_mag_open);
                //ioctl(accfp, GSENSOR_IOC_SET_DELAY, &pre_acc_delay_ms);
                //ioctl(magfp, MSENSOR_IOC_SET_DELAY, &pre_mag_delay_ms);
    		    ioctl(magfp, MSENSOR_IOCTL_SENSOR_ENABLE, &openflag);

                close(accfd_a);
	            close(magfd_a);
	            close(gyrofd_a);
            }

            // wait for virtual sensor open again
            if(ioctl(gyrfp, GYROSCOPE_IOC_GET_OPEN_STATUS, &sensor_stat)<0)
            {
                LOGD(" ST ALG  st algGYROSCOPE_IOC_GET_OPEN_STATUS failed\n");
                break;
            }
        }

        if(first_10_data<FIRST_WRONG_DATA_NUM)
            first_10_data++;

        if(openflag==0)
        {
            prev_time = 0;
            openflag = 1;
            delay_ms = 10;
            //ioctl(accfp, GSENSOR_IOC_GET_ENABLE, &pre_acc_open);
            //ioctl(magfp, ECOMPASS_IOC_GET_OPEN_STATUS, &pre_mag_open);
            //ioctl(accfp, GSENSOR_IOC_SET_ENABLE, &openflag);
		    ioctl(magfp, MSENSOR_IOCTL_SENSOR_ENABLE, &openflag);
            /*
            ioctl(accfp, GSENSOR_IOC_GET_DELAY, &pre_acc_delay_ms);
            ioctl(magfp, MSENSOR_IOC_GET_DELAY, &pre_mag_delay_ms);
            ioctl(accfp, GSENSOR_IOC_SET_DELAY, &delay_ms);
            ioctl(magfp, MSENSOR_IOC_SET_DELAY, &delay_ms);
            */

            gyrofd_a = open("/sys/bus/platform/drivers/gyroscope/gyro", O_RDONLY);
            accfd_a = open("/sys/bus/platform/drivers/gsensor/accdata", O_RDONLY);
            magfd_a = open("/sys/bus/platform/drivers/msensor/calidata1", O_RDONLY);
            LOGD(" ST ALG 000 gyrofd_a: %d, accfd_a: %d, magfd_a: %d, failed\n", gyrofd_a, accfd_a, magfd_a);
        }
#ifdef DEBUG_ENG
        cur_t = getTimestamp()/1000;
        LOGD("ST ALG current time2 = %lld", cur_t);
#endif

        if(gyrofd_a>0 && accfd_a>0 && magfd_a>0)
        {
            lseek(gyrofd_a, 0, SEEK_SET);
            read(gyrofd_a, buff_data, 32);
            sscanf(buff_data, "%d %d %d", &x, &y, &z);
            t = x;
            gx = t*0.07; //16.0;
            t = y;
            gy = t*0.07;  //16.0;
            t = z;
            gz = t*0.07;   //16.0;

#ifdef DEBUG_ENG
            cur_t = getTimestamp()/1000;
            LOGD("ST ALG current time2_gyro = %lld", cur_t);
#endif

            lseek(accfd_a, 0, SEEK_SET);
            read(accfd_a, buff_data, 32);
            sscanf(buff_data, "%d %d %d", &ax, &ay, &az);

#ifdef DEBUG_ENG
            cur_t = getTimestamp()/1000;
            LOGD("ST ALG current time2_acc = %lld", cur_t);
            //LOGD(" ST ALG  st alg===== GSensorReadRawData  %d,  %d,  %d ====\n",ax,ay,az);
#endif

            lseek(magfd_a, 0, SEEK_SET);
            read(magfd_a, buff_data, 32);
            sscanf(buff_data, "%d %d %d", &mx, &my, &mz);
            //printf(" ST ALG  st alg===== MagReadCaliData   %d,  %d,  %d  ====\n", mx, my, mz);
        }
        else
        {
            LOGD(" ST ALG gyrofd_a: %d, accfd_a: %d, magfd_a: %d, failed\n", gyrofd_a, accfd_a, magfd_a);
        }

#ifdef DEBUG_ENG
        cur_t = getTimestamp()/1000;
        LOGD("ST ALG current time3 = %lld", cur_t);
#endif
        if(first_10_data==FIRST_WRONG_DATA_NUM)
        {
            float abc;
            if(First_in_lib ==1)
            {
                int ii;
                float tempx =0.0;
                float tempy =0.0;
                float tempz =0.0;
                for(ii=0;ii<FIRST_WRONG_DATA_NUM;ii++)
                {
                    tempx = tempx + accx_first_data[ii];
                    tempy = tempy + accy_first_data[ii];
                    tempz = tempz + accz_first_data[ii];
                }
                sensor_data[0] = tempx/FIRST_WRONG_DATA_NUM;
                sensor_data[1] = tempy/FIRST_WRONG_DATA_NUM;
                sensor_data[2] = tempz/FIRST_WRONG_DATA_NUM;
                //LOGD("PNI average acc first data: %f,%f,%f,\n", sensor_data[0],sensor_data[1],sensor_data[2]);
                First_in_lib = 0;
            }
            else
            {
                //LOGD("PNI  acc after average data\n");
                abc = ax ;
                //abc = (abc-50)/1.0255;
                sensor_data[0] = abc /1000;
                abc = ay ;
                //abc = (abc+50)/0.9847;
                sensor_data[1] = abc /1000;
                abc = az ;
                //abc = (abc+250)/1.0153;
                sensor_data[2] = abc /1000;
            }
            abc = mx/1000.0;
            sensor_data[3] = abc ;//* 100 /32768;
            abc = my/1000.0;
            sensor_data[4] = abc ;//* 100 /32768;
            abc = mz/1000.0;
            sensor_data[5] = abc ;//* 100 /32768;
            sensor_data[6] = gx * 3.14159 / 180 ;//+ 0.14188;
            sensor_data[7] = gy * 3.14159 / 180 ;//- 0.232973;
            sensor_data[8] = gz * 3.14159 / 180 ;//- 0.065805;
        }
        else
        {
            float abc = ax ;
            //abc = (abc-50)/1.0255;
            accx_first_data[first_10_data] = abc /1000;
            abc = ay ;
            //abc = (abc+50)/0.9847;
            accy_first_data[first_10_data] = abc /1000;
            abc = az ;
            //abc = (abc+250)/1.0153;
            accz_first_data[first_10_data] = abc /1000;
            //LOGD("PNI  acc first data: %f,%f,%f,\n", accx_first_data[first_10_data],accy_first_data[first_10_data],accz_first_data[first_10_data]);
        }
        if(PNI_9X_opened ==0)
        {
            iNemoEngine_API_enable_9X(true);
            //iNemoEngine_API_enable_6X(false);
            PNI_9X_opened = 1;
        }
        sdata.accel[0] = sensor_data[0];//m/s^2
        sdata.accel[1] = sensor_data[1];//m/s^2
        sdata.accel[2] = sensor_data[2];//m/s^2
        sdata.magn[0] = sensor_data[3];//uT
        sdata.magn[1] = sensor_data[4];//uT
        sdata.magn[2] = sensor_data[5];//uT

        sdata.gyro[0] = sensor_data[6];//rad/sec
        sdata.gyro[1] = sensor_data[7];//rad/sec
        sdata.gyro[2] = sensor_data[8];//rad/sec

        long long time = getTimestamp();
        int64_t time_diff_ns = 10000000;
        if (prev_time > 0)
            time_diff_ns = (int64_t)(time - prev_time);

#ifdef DEBUG_ENG
        LOGD("ST ALG ino lib time = %lld", time);
        LOGD("ST ALG first_10_data = %d\n",first_10_data);
        if(time_diff_ns > 20000000)
        {
            int64_t time_diff_ms = time_diff_ns/1000;
            long long prev_time_ms = prev_time/1000;
            long long time_ms = time/1000;
            LOGD("ST ALG time = %lld, prev_time = %lld, time_diff_ms = %d\n ", time_ms, prev_time_ms, time_diff_ms);
        }
#endif

        if(first_10_data==FIRST_WRONG_DATA_NUM )
        {
#ifdef DEBUG_ENG
            //LOGD("PNI fusion into lib: %d\n",first_10_data);
            LOGD("PNI fusion input: %f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
            sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],sensor_data[8], time_diff_ns);
#endif
// 苏 勇 2014年06月06日 19:01:31            printf("PNI fusion input: %f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
// 苏 勇 2014年06月09日 08:26:11            sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],sensor_data[8], time_diff_ns);

            iNemoEngine_API_Run(time_diff_ns, &sdata);
            iNemoEngine_API_Get_Gbias(gbias);
#ifdef DEBUG_ENG
            //LOGD("PNI fusion Gbias : %f,%f,%f\n",gbias[0],gbias[1],gbias[2]);

            cur_t = getTimestamp()/1000;
            LOGD("ST ALG out lib = %lld", cur_t);
#endif

            if(gy_flag)
            {
                posturebuf[0] = gbias[0] / 3.14159 * 180.0* (-131)*1.0204;
                posturebuf[1] = gbias[1] / 3.14159 * 180.0* (-131)* 1.0204;
                posturebuf[2] = gbias[2] / 3.14159 * 180.0* (-131)*1.0204;
                posturebuf[3] = 0;

                res = ioctl(gyrfp, GYROSCOPE_IOCTL_SET_CALI, posturebuf);
            }
#ifdef DEBUG_ENG
            LOGD("PNI fusion Gbias: %f,%f,%f\n",gbias[0],gbias[1],gbias[2]);
#endif
            prev_time = time  ;
        }

        float angles[4];
        float quaternion[4];
        if(rv_flag)
        {
            iNemoEngine_API_Get_Quaternion(quaternion);

            posturebufgyro[0] = quaternion[0] * 1000000;
            posturebufgyro[1] = quaternion[1] * 1000000;
            posturebufgyro[2] = quaternion[2] * 1000000;
            posturebufgyro[3] = iStatus;
            res = ioctl(gyrfp, GYROSCOPE_IOCTL_SET_ROTATION_VECTOR, posturebufgyro);
            //LOGD(" ST ALG  st alg%s write rotation vector: %d %d %d\n", __FUNCTION__, posturebuf[0], posturebuf[1], posturebuf[2]);
        }
        if(la_flag)
        {
            iNemoEngine_API_Get_Linear_Acceleration(angles);

            posturebuf[0] = angles[0] * 1000000;
            posturebuf[1] = angles[1] * 1000000;
            posturebuf[2] = angles[2] * 1000000;
            posturebuf[3] = iStatus;
            res = ioctl(gyrfp, GYROSCOPE_IOCTL_SET_LINEAR_ACC, posturebuf);
            //LOGD(" ST ALG  st alg%s write linear acc: %d %d %d\n", __FUNCTION__, posturebuf[0], posturebuf[1], posturebuf[2]);
        }
        if(gr_flag)
        {
            iNemoEngine_API_Get_Gravity(angles);

            posturebuf[0] = angles[0] * 10000;
            posturebuf[1] = angles[1] * 10000;
            posturebuf[2] = angles[2] * 10000;
            posturebuf[3] = iStatus;
            res = ioctl(gyrfp, GYROSCOPE_IOCTL_SET_GRAVITY, posturebuf);
            //LOGD(" ST ALG  st alg%s write gravity: %d %d %d\n", __FUNCTION__, posturebuf[0], posturebuf[1], posturebuf[2]);
        }
        if(or_flag)
        {
            iNemoEngine_API_Get_Euler_Angles(angles);

            posturebuf[0] = angles[0];
            posturebuf[1] = angles[1];
            posturebuf[2] = angles[2];
            posturebuf[3] = iStatus;
            res = ioctl(gyrfp, GYROSCOPE_IOCTL_SET_ORIENTATION, posturebuf);
        }

        cur_t5 = getTimestamp()/1000;

#ifdef DEBUG_ENG
        LOGD("ST ALG current time5 = %lld", cur_t5);
#endif

        int tt = cur_t5 - cur_t1;
        delay = 10000 - tt;
        if(delay<3000)
            delay = 3000;  //delay = 20000;
#ifdef DEBUG_ENG
        LOGD("ST ALG current time --tt = %d", tt);
        LOGD("ST ALG current time --sleep delay = %d", delay);
#endif
        usleep(delay);
    }

    LOGD(" ST ALG  stalgJob finished!!\n");

    close(accfp);
    close(magfp);
    close(gyrfp);
    close(accfd_a);
    close(magfd_a);
    close(gyrofd_a);

    return 0;
}

