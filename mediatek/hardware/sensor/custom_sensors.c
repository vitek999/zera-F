/******************* V:\szgit\82jb5\mediatek\hardware\sensor\hwmsen\custom_sensors.c ************************
    创建时间: 2013年08月01日  10:36:41
    主要功能: 订制各种sensor
    作    者: 苏 勇 
    版    本: 初始版本   1.0
    其    它: 主要功能:
              根据驱动给出的sys文件读取各种sensor的名字,根据sensor的名字
              和类型动态产生pSensor,供外部使用
******************************************************************/
//头文件包含
#include <stdio.h>
#include <stdlib.h>
#include <hardware/sensors.h>
#include <linux/hwmsensor.h>
#include <cutils/log.h>
#include "hwmsen_chip_info.h"
#include "hwmsen_custom.h"
#include "custom_sensors.h"
#define LOG_TAG "CUSTOM_SENSOR"

//静态函数声明
static int	GetIndex(const char *name, int type);
static const char	*GetSensorName(int type);
static void GetSensorNameHelp(char *sensorname, const char *fileName);

//静态变量声明

// 整个sensor的表格,前面的一个是在系统文件中得到的名字,后面的是sensor的类型 苏 勇 2013年08月01日 14:11:04
// 要保持顺序和sSensorListAllInOne一致 苏 勇 2013年08月01日 14:11:46
static const CUSTOM_SENSOR_TABLE customSensorTable[] =
{
	{"mmc3516x Chip",SENSOR_TYPE_ORIENTATION},
	{"mmc3516x Chip",SENSOR_TYPE_MAGNETIC_FIELD},
	{"MPU6880 Chip",SENSOR_TYPE_ACCELEROMETER},
	{"KXTIK1004 Chip",SENSOR_TYPE_ACCELEROMETER},
	{"TMD2772 Chip",SENSOR_TYPE_PROXIMITY},
	{"TMD2772 Chip",SENSOR_TYPE_LIGHT},
	{"MPU6880 Chip",SENSOR_TYPE_GYROSCOPE},
	{"MC3250 Chip",SENSOR_TYPE_ACCELEROMETER},
	{"lps331ap Chip",SENSOR_TYPE_PRESSURE},
	{"STK3X1X Chip",SENSOR_TYPE_PROXIMITY},
	{"STK3X1X Chip",SENSOR_TYPE_LIGHT},
	{"EPL2182 Chip",SENSOR_TYPE_PROXIMITY},
	{"EPL2182 Chip",SENSOR_TYPE_LIGHT},
	{"LSM330 Chip",SENSOR_TYPE_ACCELEROMETER},
	{"L3GD20 Chip",SENSOR_TYPE_GYROSCOPE},
	{"st480 Chip",SENSOR_TYPE_ORIENTATION},
	{"st480 Chip",SENSOR_TYPE_MAGNETIC_FIELD},
	{"KXCJK_1013 Chip",SENSOR_TYPE_ACCELEROMETER},
	{"LIS3DH Chip",SENSOR_TYPE_ACCELEROMETER},
};

// 这里列出所有可能用到的sensor 苏 勇 2013年08月01日 14:12:31
// 要保持顺序和customSensorTable一致 苏 勇 2013年08月01日 14:11:46
const static struct sensor_t sSensorListAllInOne[] =
{
	{
		.name		= "mmc3516x Orientation sensor",
		.vendor		= "MEMSEC",
		.version	= 1,
		.handle		= ID_ORIENTATION,
		.type		= SENSOR_TYPE_ORIENTATION,		  
		.maxRange	  = ORIENTATION_RANGE,	//360.0f,
		.resolution = ORIENTATION_RESOLUTION,	//1.0f,
		.power		= ORIENTATION_POWER,	//0.25f,
		.reserved	= {}
	},

	{
		.name		= "mmc3516x 3-axis Magnetic Field sensor",
		.vendor		= "MEMSEC",
		.version	= 1,
		.handle		= ID_MAGNETIC,
		.type		= SENSOR_TYPE_MAGNETIC_FIELD,
		.maxRange	= MAGNETOMETER_RANGE,	//600.0f,
		.resolution = MAGNETOMETER_RESOLUTION,	//0.0016667f,
		.power		= MAGNETOMETER_POWER,	//0.25f,
		.reserved	= {}
	},

	{
		.name		= "MPU6880 3-axis Accelerometer",
		.vendor		= "InvenSense",
		.version	= 1,
		.handle		= ID_ACCELEROMETER,
		.type		= SENSOR_TYPE_ACCELEROMETER,
		.maxRange	= ACCELEROMETER_RANGE,	//32.0f,
		.resolution = ACCELEROMETER_RESOLUTION, //4.0f/1024.0f,
		.power		= ACCELEROMETER_POWER,	//130.0f/1000.0f,
		.reserved	= {}
	},

	{
		.name		= "KXTIK1004 3-axis Accelerometer",
		.vendor		= "Kionix",
		.version	= 1,
		.handle		= ID_ACCELEROMETER,
		.type		= SENSOR_TYPE_ACCELEROMETER,
		.maxRange	= ACCELEROMETER_RANGE,	//32.0f,
		.resolution = ACCELEROMETER_RESOLUTION, //4.0f/1024.0f,
		.power		= ACCELEROMETER_POWER,	//130.0f/1000.0f,
		.reserved	= {}
	},
	
	{
		.name		= "tmd2772 Proximity Sensor",
		.vendor		= "Capella",
		.version	= 1,
		.handle		= ID_PROXIMITY,
		.type		= SENSOR_TYPE_PROXIMITY,		
		.maxRange	= PROXIMITY_RANGE,	//1.00f,
		.resolution = PROXIMITY_RESOLUTION, //1.0f,
		.power		= PROXIMITY_POWER,	//0.13f,
		.reserved	= {}
	},

	{
		.name		= "tmd2772 Light Sensor",
		.vendor		= "Capella",
		.version	= 1,
		.handle		= ID_LIGHT,
		.type		= SENSOR_TYPE_LIGHT,		
		.maxRange	= LIGHT_RANGE,	//10240.0f,
		.resolution = LIGHT_RESOLUTION, //1.0f,
		.power		= LIGHT_POWER,	//0.13f,
		.reserved	= {}
	},

	{
		.name		= "MPU6880 Gyroscope sensor",
		.vendor		= "InvenSense",
		.version	= 1,
		.handle		= ID_GYROSCOPE,
		.type		= SENSOR_TYPE_GYROSCOPE,		
		.maxRange	= GYROSCOPE_RANGE,	//34.91f,
		.resolution = GYROSCOPE_RESOLUTION, //0.0107f,
		.power		= GYROSCOPE_POWER,	//6.1f,
		.reserved	= {}
	},

	{
		.name		= "MC3250 3-axis Accelerometer",
		.vendor		= "Mcube",
		.version	= 1,
		.handle		= ID_ACCELEROMETER,
		.type		= SENSOR_TYPE_ACCELEROMETER,
		.maxRange	= ACCELEROMETER_RANGE,	//32.0f,
		.resolution = ACCELEROMETER_RESOLUTION, //4.0f/1024.0f,
		.power		= ACCELEROMETER_POWER,	//130.0f/1000.0f,
		.reserved	= {}
	},
	
	{ 
		.name       = "lps331ap Pressure sensor",
		.vendor     = "ST",
		.version    = 1,
		.handle     = ID_PRESSURE,
		.type       = SENSOR_TYPE_PRESSURE,
		.maxRange   = PRESSURE_RANGE,//360.0f,
		.resolution = PRESSURE_RESOLUTION,//1.0f,
		.power      = PRESSURE_POWER,//0.25f,
		.reserved   = {}
	},

	{
		.name		= "STK3X1X Proximity Sensor",
		.vendor		= "Sitronix",
		.version	= 1,
		.handle		= ID_PROXIMITY,
		.type		= SENSOR_TYPE_PROXIMITY,		
		.maxRange	= PROXIMITY_RANGE,	//1.00f,
		.resolution = PROXIMITY_RESOLUTION, //1.0f,
		.power		= PROXIMITY_POWER,	//0.13f,
		.reserved	= {}
	},
	
	{
		.name		= "STK3X1X Light Sensor",
		.vendor		= "Sitronix",
		.version	= 1,
		.handle		= ID_LIGHT,
		.type		= SENSOR_TYPE_LIGHT,		
		.maxRange	= LIGHT_RANGE,	//10240.0f,
		.resolution = LIGHT_RESOLUTION, //1.0f,
		.power		= LIGHT_POWER,	//0.13f,
		.reserved	= {}
	},
	
	{
		.name		= "EPL2182 Proximity Sensor",
		.vendor		= "ELAN",
		.version	= 1,
		.handle		= ID_PROXIMITY,
		.type		= SENSOR_TYPE_PROXIMITY,		
		.maxRange	= PROXIMITY_RANGE,	//1.00f,
		.resolution = PROXIMITY_RESOLUTION, //1.0f,
		.power		= PROXIMITY_POWER,	//0.13f,
		.reserved	= {}
	},
	
	{
		.name		= "EPL2182 Light Sensor",
		.vendor		= "ELAN",
		.version	= 1,
		.handle		= ID_LIGHT,
		.type		= SENSOR_TYPE_LIGHT,		
		.maxRange	= LIGHT_RANGE,	//10240.0f,
		.resolution = LIGHT_RESOLUTION, //1.0f,
		.power		= LIGHT_POWER,	//0.13f,
		.reserved	= {}
	},

	{
		.name		= "LSM330 3-axis Accelerometer",
		.vendor		= "STMicroelectronics",
		.version	= 1,
		.handle		= ID_ACCELEROMETER,
		.type		= SENSOR_TYPE_ACCELEROMETER,
		.maxRange	= ACCELEROMETER_RANGE,	//32.0f,
		.resolution = ACCELEROMETER_RESOLUTION, //4.0f/1024.0f,
		.power		= ACCELEROMETER_POWER,	//130.0f/1000.0f,
		.reserved	= {}
	},

	{
		.name		= "L3GD20 Gyroscope sensor",
		.vendor		= "STMicroelectronics",
		.version	= 1,
		.handle		= ID_GYROSCOPE,
		.type		= SENSOR_TYPE_GYROSCOPE,		
		.maxRange	= GYROSCOPE_RANGE,	//34.91f,
		.resolution = GYROSCOPE_RESOLUTION, //0.0107f,
		.power		= GYROSCOPE_POWER,	//6.1f,
		.reserved	= {}
	},

	{
		.name		= "st480 Orientation sensor",
		.vendor		= "Senodia",
		.version	= 1,
		.handle		= ID_ORIENTATION,
		.type		= SENSOR_TYPE_ORIENTATION,		  
		.maxRange	  = ORIENTATION_RANGE,	//360.0f,
		.resolution = ORIENTATION_RESOLUTION,	//1.0f,
		.power		= ORIENTATION_POWER,	//0.25f,
		.reserved	= {}
	},

	{
		.name		= "st480 3-axis Magnetic Field sensor",
		.vendor		= "Senodia",
		.version	= 1,
		.handle		= ID_MAGNETIC,
		.type		= SENSOR_TYPE_MAGNETIC_FIELD,
		.maxRange	= MAGNETOMETER_RANGE,	//600.0f,
		.resolution = MAGNETOMETER_RESOLUTION,	//0.0016667f,
		.power		= MAGNETOMETER_POWER,	//0.25f,
		.reserved	= {}
	},

	{
		.name		= "KXCJK_1013 3-axis Accelerometer",
		.vendor		= "Kionix",
		.version	= 1,
		.handle		= ID_ACCELEROMETER,
		.type		= SENSOR_TYPE_ACCELEROMETER,
		.maxRange	= ACCELEROMETER_RANGE,	//32.0f,
		.resolution = ACCELEROMETER_RESOLUTION, //4.0f/1024.0f,
		.power		= ACCELEROMETER_POWER,	//130.0f/1000.0f,
		.reserved	= {}
	},

	{
		.name		= "LIS3DH 3-axis Accelerometer",
		.vendor		= "STMicroelectronics",
		.version	= 1,
		.handle		= ID_ACCELEROMETER,
		.type		= SENSOR_TYPE_ACCELEROMETER,
		.maxRange	= ACCELEROMETER_RANGE,	//32.0f,
		.resolution = ACCELEROMETER_RESOLUTION, //4.0f/1024.0f,
		.power		= ACCELEROMETER_POWER,	//130.0f/1000.0f,
		.reserved	= {}
	}


};

#if defined (CKT_USER_INEMOENGINE)  // 苏 勇 2014年06月06日 16:55:34
// 所有的虚拟传感器,会直接添加进去,无论是不是实际存在 苏 勇 2014年05月30日 14:57:13
const static struct sensor_t sSensorListAllVisual[] =
{
	{
		.name		= "rotation vector sensor",
		.vendor		= "ckt",
		.version	= 1,
		.handle		= ID_ROTATION_VECTOR,
		.type		= SENSOR_TYPE_ROTATION_VECTOR,
		.maxRange	= 1.0,	//600.0f,
		.resolution = 0.001,	//0.0016667f,
		.power		= 6.21,	//0.25f,
		.reserved	= {}
	},

	{
		.name		= "gravity sensor",
		.vendor		= "ckt",
		.version	= 1,
		.handle		= ID_GRAVITY,
		.type		= SENSOR_TYPE_GRAVITY,
		.maxRange	= 19.6,	//600.0f,
		.resolution = 19.6/1024,	//0.0016667f,
		.power		= 6.21,	//0.25f,
		.reserved	= {}
	},


	{
		.name		= "linnera acceleration sensor",
		.vendor		= "ckt",
		.version	= 1,
		.handle		= ID_LINEAR_ACCELERATION,
		.type		= SENSOR_TYPE_LINEAR_ACCELERATION,
		.maxRange	= 19.6,	//600.0f,
		.resolution = 19.6/1024,	//0.0016667f,
		.power		= 6.21,	//0.25f,
		.reserved	= {}
	}

};
#endif /* CKT_USER_INEMOENGINE */

//全局变量定义
struct sensor_t pSensor[MAX_NUM_SENSOR] = {0};
/******************* 函数名: GetSensorNameHelp **********************
    创造时间: 2013年08月01日 14:13:31
    函数功能: 从文件系统中得到sensor的名字的帮助文件,供其他得到名字的
              函数调用
    函数参数: 
        sensorname :输出,sensor的名字
        fileName :系统文件的路径
    返回类型: 无 
    其    它: 
    更新历史: 
      作  者		时间				版本		变更: 
    =============================================================
	 苏 勇    	2013年08月01日 14:13:31	1.00		初始版本
****************************************************************/
static void GetSensorNameHelp(char *sensorname, const char *fileName)
{
	FILE *fd = 0;
	sensorname[MAX_SENSOR_NAME_SIZE - 1] = 0;
	if((fd = fopen(fileName, "r")) <= 0)
	{
		CUSTOM_SENSOR_LOG("Can't open file : %s\n", fileName);
		sensorname = "";
	}
	else
	{
		if(fgets(sensorname, MAX_SENSOR_NAME_SIZE - 1, fd) != NULL)
		{
			if(sensorname[strlen(sensorname) - 1] == '\n')
			{
				sensorname[strlen(sensorname) - 1] = 0;
			}
		}
		else
		{
			sensorname = "";
		}
	}

	CUSTOM_SENSOR_LOG("%s the name is : %s\n", fileName, sensorname);
}

/******************* 函数名: GetSensorNameForProximity **********************
    创造时间: 2013年08月01日 14:14:24
    函数功能: 得到接近传感器的名字
    函数参数: 
        sensorname :输出 接近传感器的名字
    返回类型: 无 
    其    它: 如果有合适的属性文件后,可以使用类似其他的传感器方法,本函数可以去除
    更新历史: 
      作  者		时间				版本		变更: 
    =============================================================
	 苏 勇    	2013年08月01日 14:14:24	1.00		初始版本
****************************************************************/
/*
static void GetSensorNameForProximity(char *sensorname)
{
	strcpy(sensorname, "tmd2772");
	CUSTOM_SENSOR_LOG("Proximity the name is : %s\n", sensorname);
}
*/
/******************* 函数名: GetSensorNameForLight **********************
    创造时间: 2013年08月01日 14:14:50
    函数功能: 得到光传感器的名字
    函数参数: 
        sensorname :输出 光传感器的名字
    返回类型: 无 
    其    它: 如果有合适的属性文件后,可以使用类似其他的传感器方法,本函数可以去除
    更新历史: 
      作  者		时间				版本		变更: 
    =============================================================
	 苏 勇    	2013年08月01日 14:14:50	1.00		初始版本
****************************************************************/
/*
static void GetSensorNameForLight(char *sensorname)
{
	strcpy(sensorname, "tmd2772");
	CUSTOM_SENSOR_LOG("Light the name is : %s\n", sensorname);
}
*/
/******************* 函数名: GetSensorName **********************
    创造时间: 2013年08月01日 14:15:15
    函数功能: 得到各种传感器的名字
    函数参数: 
        type :传感器的类型
    返回类型: char * 含有传感器名字的指针
    其    它: 如果有其他传感器类型,注意添加
    更新历史: 
      作  者		时间				版本		变更: 
    =============================================================
	 苏 勇    	2013年08月01日 14:15:15	1.00		初始版本
****************************************************************/
static const char *GetSensorName(int type)
{
	static char sensorname[MAX_SENSOR_NAME_SIZE] ={0};
	memset(sensorname, 0, sizeof(sensorname));
	switch(type)
	{
	case SENSOR_TYPE_ORIENTATION:
		GetSensorNameForOrientation(sensorname);
		break;

	case SENSOR_TYPE_MAGNETIC_FIELD:
		GetSensorNameForMagnetic(sensorname);
		break;

	case SENSOR_TYPE_ACCELEROMETER:
		GetSensorNameForAccelerometer(sensorname);
		break;

	case SENSOR_TYPE_PROXIMITY:
		GetSensorNameForProximity(sensorname);
		break;

	case SENSOR_TYPE_LIGHT:
		GetSensorNameForLight(sensorname);
		break;

	case SENSOR_TYPE_GYROSCOPE:
		GetSensorNameForGyroscope(sensorname);
		break;

	case SENSOR_TYPE_PRESSURE:
		GetSensorNameForBarometer(sensorname);
		break;
		
	case SENSOR_TYPE_TEMPERATURE:
		break;
	default:
		break;
	}

	return sensorname;
}

/******************* 函数名: GetIndex **********************
    创造时间: 2013年08月01日 14:16:11
    函数功能: 根据名字得到传感器在customSensorTable的索引位置
    函数参数: 
        name :传感器的名字
        type :传感器的类型
    返回类型: int 在名字和类型都匹配的时候返回在customSensorTable中的
                  索引
                  在不匹配的时候返回-1
    其    它: 
    更新历史: 
      作  者		时间				版本		变更: 
    =============================================================
	 苏 勇    	2013年08月01日 14:16:11	1.00		初始版本
****************************************************************/
static int GetIndex(const char *name, int type)
{
	int i;
	int size = sizeof(customSensorTable) / sizeof(customSensorTable[0]);
	for(i = 0; i < size; i++)
	{
		if(!strcmp((char const *) name, customSensorTable[i].name) && type == customSensorTable[i].type)
		{
			break;
		}
	}

	CUSTOM_SENSOR_LOG("GetIndex the index is : %d\n", i < size ? i : -1);
	return i < size ? i : -1;
}

/******************* 函数名: GenpSensor **********************
    创造时间: 2013年08月01日 14:17:14
    函数功能: 根据对应的名字,产生/动态填充pSensor
    函数参数: 无
    返回类型: int 产生的sensor的总共个数
    其    它: 
    更新历史: 
      作  者		时间				版本		变更: 
    =============================================================
	 苏 勇    	2013年08月01日 14:17:14	1.00		初始版本
****************************************************************/
int GenpSensor(void)
{
	int i;
	char *p;
	int table[SENSOR_TYPE_AMBIENT_TEMPERATURE - SENSOR_TYPE_ACCELEROMETER + 1];
	static int	oldnum = 0;
	int num = 0;
	int index;
	static char hadInited = 0;
	CUSTOM_SENSOR_LOG("entry\n");
	if(hadInited)
	{
		CUSTOM_SENSOR_LOG("had inited exit %d\n", oldnum);
		return oldnum;
	}

	for(i = SENSOR_TYPE_ACCELEROMETER; i <= SENSOR_TYPE_AMBIENT_TEMPERATURE; i++)
	{
		p = GetSensorName(i);
		index = GetIndex(p, i);
		if(index != -1)
		{
			if(index > sizeof(sSensorListAllInOne) / sizeof(sSensorListAllInOne[0]) - 1)
			{
				ALOGE("[%s] ERROR at %d index=%d is too big", __FUNCTION__, __LINE__, index);
			}
			else
			{
				memcpy((char *) (pSensor + num), (char *) (&sSensorListAllInOne[index]), sizeof(sSensorListAllInOne[0]));
				num++;
			}
		}

		if(num > MAX_NUM_SENSOR)
		{
			ALOGE("[%s] ERROR at %d num=%d is too big", __FUNCTION__, __LINE__, num);
		}
	}


	
#if defined (CKT_USER_INEMOENGINE)  // 苏 勇 2014年06月06日 16:55:52
	for(i=0 ;i < sizeof(sSensorListAllVisual)/sizeof(sSensorListAllVisual[0]); i++)
	{
		memcpy((char *) (pSensor + num), (char *) (&sSensorListAllVisual[i]), sizeof(sSensorListAllVisual[0]));
		num++;
		ALOGE("[%s] asdfasdf at %d num=%d is too big", __FUNCTION__, __LINE__, num);
	}
#endif /* CKT_USER_INEMOENGINE */

	
	CUSTOM_SENSOR_LOG("%d in pSensor\n", num);
	hadInited = 1;
	oldnum = num;
	return num;
}

