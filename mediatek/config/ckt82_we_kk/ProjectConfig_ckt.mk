CKT_AUTO_ADD_GLOBAL_DEFINE_BY_NAME = CKT_INTERPOLATION CKT_SUPPORT_AUTOTEST_MODE CKT_LOW_POWER_SUPPORT RESPIRATION_LAMP TEMPERATURE_CONTROL_CHARGING CKT_USER_INEMOENGINE
CKT_AUTO_ADD_GLOBAL_DEFINE_BY_NAME_VALUE =PROJ_NAME CUST_NAME SOFTCODE USB_MANUFACTURER_STRING USB_PRODUCT_STRING USB_STRING_SERIAL_IDX CUSTOM_EXIF_STRING_MAKE CUSTOM_EXIF_STRING_MODEL CUSTOM_EXIF_STRING_SOFTWARE CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME CUSTOM_BTMTK_ANDROID_DEFAULT_LOCAL_NAME
CKT_AUTO_ADD_GLOBAL_DEFINE_BY_VALUE = 
#############################
#############################
#############################

#��Ŀ����ض���
PROJ_NAME = ZERA01A
CUST_NAME = VOBIS
SOFTCODE = S10A
BASEVERNO =204 
#############################
#����������ro.product.model
CKT_PRODUCT_MODEL=CKT_$(strip $(PROJ_NAME) )
#����������ȱʡʱ��persist.sys.timezone
TIMEZONE=Europe/Moscow


############usb���#################
USB_MANUFACTURER_STRING=$(strip $(CUST_NAME) )
USB_PRODUCT_STRING=$(strip $(CKT_PRODUCT_MODEL) )
USB_STRING_SERIAL_IDX=$(strip $(USB_PRODUCT_STRING) )

############exif���#################
CUSTOM_EXIF_STRING_MAKE=$(strip $(CUST_NAME) )
CUSTOM_EXIF_STRING_MODEL=$(strip $(PROJ_NAME) )
CUSTOM_EXIF_STRING_SOFTWARE=""

############bt���#################
CUSTOM_BTMTK_ANDROID_DEFAULT_LOCAL_NAME =$(strip $(PROJ_NAME) )_BT
CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME=$(strip $(PROJ_NAME) )_DEVICE

#############################
#���ܵĿ���,�ᵼ�뵽mediatek/source/frameworks/featureoption/java/com/mediatek/featureoption/FeatureOption.java
#�޸ĵ�ʱ��ע��,�� mediatek/build/tools/javaoption.pm��������ģ��
#����ע�����enableֻ������yes,������������
TESTA = yes
TESTB = no
TESTC = testc_none

#############################

#���Ҫ�̶��汾��,����������,����ע�͵���,����������!!!
#CKT_BUILD_VERNO = PANDORA-S0A_CKT_L2EN_111_111111
#CKT_BUILD_INTERNAL_VERNO =PANDORA-S0A_CKT_L2EN_111_111111111111

#############################
#����ͷ�����ֵ
CKT_INTERPOLATION = no
#����ģʽ�Զ����Կ���
CKT_SUPPORT_AUTOTEST_MODE = no
#MTK��low power ����
CKT_LOW_POWER_SUPPORT = no
#LED��ɫ�����Ƚ���,
RESPIRATION_LAMP = no

# 4.35V��ء���
# 1. 0�����£��������磻
# 2. 0��-10�棬������0.1C�ҡ�400mA��������Ƶ�ѹ4.2V��
# 3. 10��-45�棬��������0.5C��������Ƶ�ѹ4.35V��
# 4. 45�����ϣ��������硣
#  
# 4.2V��ء��������û�Ϊ��Ŀ��Ҫ��
# 1. -10��~0�棬��������200mA����ֹ��ѹ4.0V
# 2. 0��~50�棬������磨doro��Ŀ�ͻ�Ҫ��45��ֹͣ��磩
# 3. 50������ֹͣ��磨doro��Ŀ�ͻ�Ҫ��45��ֹͣ��磩
#  
# �������ڵ�����˵ĳ���ѹ���ܳ�������ѹ����4.2V��س���ѹ���ܸ���4.2V��4.35V��س���ѹ���ܸ���4.35V��
#  
# 55�����ϸ��¸澯��ʾ
# -20�����µ��¸澯��ʾ
#  
# ��ʾ���в�Ҫ���֡���ع��ȡ���������ʾ�á������ֻ����ȡ�����������������л��ͣ��������õ�ء����õ�ص� 
# ���ܳ���Ҫ���û����Ƴ���ء�����ʾ����������������õ�صĻ���
TEMPERATURE_CONTROL_CHARGING = yes


#���Զ��汾���л�����
CKT_VERSION_AUTO_SWITCH=yes
export CKT_VERSION_AUTO_SWITCH

#ʹ��st��fussiion����
CKT_USER_INEMOENGINE=yes
export CKT_USER_INEMOENGINE



#ckt helin 20131210 add FlashLight apk to System
CKT_APP_FLASHLIGHT = no
export CKT_APP_FLASHLIGHT








































###########����Ϊ�����Ķ���,һ�㲻��Ҫ���
_CKT_BUILD_VERNO  = $(strip $(PROJ_NAME) )-$(strip $(SOFTCODE) )_$(strip $(CUST_NAME) )_L$(words $(subst hdpi, ,$(strip $(MTK_PRODUCT_LOCALES))))$(word 1,$(subst _, ,$(subst zh_TW,TR,$(subst zh_CN,SM,$(strip $(MTK_PRODUCT_LOCALES))))))_$(strip $(BASEVERNO))

DATA_FOR_VERO=$(shell date +%y%m%d)
DATA_FOR_INTERNAL_VERO=$(shell date +%y%m%d%H%M%S)

CKT_BUILD_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_VERO)))
CKT_BUILD_INTERNAL_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_INTERNAL_VERO)))
#############################

