#ifndef CUSTOM_SENSORS_H
#define CUSTOM_SENSORS_H

//宏定义
#define DEBUG_FOR_CUSTOM_SENSOR	(0)
#define MAX_SENSOR_NAME_SIZE	(33)
// 各种属性文件的路径 苏 勇 2013年08月01日 14:21:42
#define NAME_FILE_FOR_ORIENTATION			"/sys/bus/platform/drivers/msensor/chipinfo"
#define NAME_FILE_FOR_MAGNETIC				"/sys/bus/platform/drivers/msensor/chipinfo"
#define NAME_FILE_FOR_ACCELEROMETER			"/sys/bus/platform/drivers/gsensor/chipinfo"
#define NAME_FILE_FOR_PROXIMITY				"/sys/bus/platform/drivers/als_ps/chipinfo"
#define NAME_FILE_FOR_LIGHT					"/sys/bus/platform/drivers/als_ps/chipinfo"
#define NAME_FILE_FOR_GYROSCOPE				"/sys/bus/platform/drivers/gyroscope/chipinfo"
#define NAME_FILE_FOR_BAROMETER				"/sys/bus/platform/drivers/barometer/chipinfo"

#define GetSensorNameForOrientation(a)		GetSensorNameHelp(a, NAME_FILE_FOR_ORIENTATION)
#define GetSensorNameForMagnetic(a)			GetSensorNameHelp(a, NAME_FILE_FOR_MAGNETIC)
#define GetSensorNameForAccelerometer(a) 	GetSensorNameHelp(a, NAME_FILE_FOR_ACCELEROMETER)
#define GetSensorNameForProximity(a)               GetSensorNameHelp(a,NAME_FILE_FOR_PROXIMITY)
#define GetSensorNameForLight(a)                      GetSensorNameHelp(a,NAME_FILE_FOR_LIGHT)
#define GetSensorNameForGyroscope(a)     	       GetSensorNameHelp(a, NAME_FILE_FOR_GYROSCOPE)
#define GetSensorNameForBarometer(a)     	       GetSensorNameHelp(a, NAME_FILE_FOR_BAROMETER)

#if (DEBUG_FOR_CUSTOM_SENSOR)	// 苏 勇 2013年08月01日 09:46:18
#define CUSTOM_SENSOR_LOG(fmt, arg...)		   \
	do										   \
	{										   \
		ALOGD("[%s]"fmt, __FUNCTION__, ##arg); \
	} while(0)
#else
#define CUSTOM_SENSOR_LOG
#endif /* DEBUG_FOR_CUSTO_SENSOR */

 // 以下的定义来自mtk原始文件,注意对不同的sensor,参数有可能要调整 苏 勇 2013年08月01日 14:23:11
#ifndef ACCELEROMETER
#define ACCELEROMETER			"ACCELEROMETER"
#define ACCELEROMETER_VENDER	"MTK"
#endif

#ifndef ACCELEROMETER_RANGE
#define ACCELEROMETER_RANGE 32.0f
#endif

#ifndef ACCELEROMETER_RESOLUTION
#define ACCELEROMETER_RESOLUTION	4.0f / 1024.0f
#endif

#ifndef ACCELEROMETER_POWER
#define ACCELEROMETER_POWER 130.0f / 1000.0f
#endif

#ifndef PROXIMITY
#define PROXIMITY			"PROXIMITY"
#define PROXIMITY_VENDER	"MTK"
#endif

#ifndef PROXIMITY_RANGE
#define PROXIMITY_RANGE 1.00f
#endif

#ifndef PROXIMITY_RESOLUTION
#define PROXIMITY_RESOLUTION	1.0f
#endif

#ifndef PROXIMITY_POWER
#define PROXIMITY_POWER 0.13f
#endif

#ifndef LIGHT
#define LIGHT			"LIGHT"
#define LIGHT_VENDER	"MTK"
#endif

#ifndef LIGHT_RANGE
#define LIGHT_RANGE 10240.0f
#endif

#ifndef LIGHT_RESOLUTION
#define LIGHT_RESOLUTION	1.0f
#endif

#ifndef LIGHT_POWER
#define LIGHT_POWER 0.13f
#endif

#ifndef MAGNETOMETER
#define MAGNETOMETER		"MAGNETOMETER"
#define MAGNETOMETER_VENDER "MTK"
#endif

#ifndef MAGNETOMETER_RANGE
#define MAGNETOMETER_RANGE	600.0f
#endif

#ifndef MAGNETOMETER_RESOLUTION
#define MAGNETOMETER_RESOLUTION 0.0016667f
#endif

#ifndef MAGNETOMETER_POWER
#define MAGNETOMETER_POWER	0.25f
#endif

#ifndef ORIENTATION
#define ORIENTATION			"ORIENTATION"
#define ORIENTATION_VENDER	"MTK"
#endif

#ifndef ORIENTATION_RANGE
#define ORIENTATION_RANGE	360.0f
#endif

#ifndef ORIENTATION_RESOLUTION
#define ORIENTATION_RESOLUTION	1.0f
#endif

#ifndef ORIENTATION_POWER
#define ORIENTATION_POWER	0.25f
#endif

#ifndef GYROSCOPE
#define GYROSCOPE			"GYROSCOPE"
#define GYROSCOPE_VENDER	"MTK"
#endif

#ifndef GYROSCOPE_RANGE
#define GYROSCOPE_RANGE 34.91f
#endif

#ifndef GYROSCOPE_RESOLUTION
#define GYROSCOPE_RESOLUTION	0.0107f
#endif

#ifndef GYROSCOPE_POWER
#define GYROSCOPE_POWER 6.1f
#endif

#ifndef PRESSURE
#define PRESSURE		"PRESSURE"
#define PRESSURE_VENDER "MTK"
#endif

#ifndef PRESSURE_RANGE
#define PRESSURE_RANGE	1100.0f
#endif

#ifndef PRESSURE_RESOLUTION
#define PRESSURE_RESOLUTION 100.0f
#endif

#ifndef PRESSURE_POWER
#define PRESSURE_POWER	0.5f
#endif

#ifndef TEMPURATURE
#define TEMPURATURE			"TEMPURATURE"
#define TEMPURATURE_VENDER	"MTK"
#endif

#ifndef TEMPURATURE_RANGE
#define TEMPURATURE_RANGE	"TEMPURATURE"
#endif

#ifndef TEMPURATURE_RESOLUTION
#define TEMPURATURE_RESOLUTION	"TEMPURATURE"
#endif

#ifndef TEMPURATURE_POWER
#define TEMPURATURE_POWER	"TEMPURATURE"
#endif

typedef struct sensor_table
{
	const char	*name; // 在系统文件中的名字 苏 勇 2013年08月01日 14:24:25
	int type; // 传感器的类型 苏 勇 2013年08月01日 14:24:30
	// 苏 勇 2013年08月01日 11:20:43    int             index; // 在中sSensorListAllInOne的位置,目前没有使用,采用的是位置一致来保证的 苏 勇 2013年08月01日 14:25:04
} CUSTOM_SENSOR_TABLE;

//函数定义
int GenpSensor(void);

//全局变量声明
#endif /* CUSTOM_SENSORS_H */

