/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_LIGHT_SENSOR_H
#define ANDROID_LIGHT_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensordebug.h"
#include "nusensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"
#include <linux/hwmsensor.h> // by fwq


/*****************************************************************************/

struct input_event;

//static uint32_t g_active_sensors;


struct sensor_delay 
{
   int handle;
   uint32_t delay;
};

class Hwmsen : public SensorBase {
	
	enum {
        Accelerometer    = 0,
        MagneticField    = 1,
        Orientation      = 2, 
        Gyro             = 3,
        light            = 4,
        proximity        = 5,
        pressure         = 6,
#if defined (CKT_USER_INEMOENGINE)  // 苏 勇 2014年06月06日 16:54:06
        rotationvector,
        gravity,
        lineraacc,
        
        numSensors   =11     // 这里的11是临时解决的方法,否则无法对rotatiionvector进行delay的设置 苏 勇 2014年05月30日 15:04:03
#else
		numSensors       = 7,//hwmsen driver process 6 device data
#endif /* CKT_USER_INEMOENGINE */
    };
	
	uint32_t mActiveSensors;
    int mEnabled;
    InputEventCircularReader mInputReader;
    
    bool mHasPendingEvent;

	uint64_t mDelays[numSensors];
	uint32_t mPendingMask;
	

    float indexToValue(size_t index) const;
    void processEvent(int code, int value);
	int update_delay(int what);
	

public:
	
	sensors_event_t mPendingEvents[numSensors];
	SensorDebugObject *mHwmSensorDebug;
	
            Hwmsen();
    virtual ~Hwmsen();
    virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents() const;
    virtual int enable(int32_t handle, int enabled);
	virtual int setDelay(int32_t handle, int64_t ns);
	int enableNoHALDataAcc(int en);
};

/*****************************************************************************/

#endif  // ANDROID_LIGHT_SENSOR_H
