#ifndef __LINUX_HY46XX_TS_H__
#define __LINUX_HY46XX_TS_H__

//RST GPIO Setting
#define HY46XX_RESET_PIN	64//set reset gpio
#define HY46XX_RESET_PIN_NAME	"hy46xx-reset"
/************************************************************/
//#define HY46XX_FW_UPDATE_ENABLE
//#define HYS_APK_DEBUG			//must define HY46XX_FW_UPDATE_ENABLE at the same time

//#define LINUX_OS

/************************************************************/
#define HY46XX_NAME	"hy46xx_ts"
#define CFG_MAX_TOUCH_POINTS	5
#define RESOLUTION_X			720
#define RESOLUTION_Y			1280

#define HY_PRESS				0x08
#define HY_MAX_ID				0x0F
#define HY_TOUCH_STEP			6
#define HY_TOUCH_X_H_POS		3
#define HY_TOUCH_X_L_POS		4
#define HY_TOUCH_Y_H_POS		5
#define HY_TOUCH_Y_L_POS		6
#define HY_TOUCH_EVENT_POS		3
#define HY_TOUCH_ID_POS			5
#define HY_DRIVER_VERSION          "V1.0<2021/06/16>"
#define HY_I2C_NAME                "Hycon-TS"
#define POINT_READ_BUF			(3 + HY_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

#define HY46XX_REG_FW_VER		0xA6
#define HY46XX_REG_WORK_MODE	0xA5
#ifdef HYS_APK_DEBUG
#define HY_RW_IIC_DRV  "hy_rw_iic_drv"
#define HY_RW_IIC_DRV_MAJOR 	210    /*Ԥ����ft_rw_iic_drv�����豸��*/
#define HY_I2C_RDWR_MAX_QUEUE 	36
#define HY_I2C_SLAVEADDR   		11
#define HY_I2C_RW          		12
#define HY_UPDATE_FW			13
#define HY_BOOT_VER				14
#define HY_CMD_ResetTP			15
#endif
#ifdef HY46XX_FW_UPDATE_ENABLE
#define    HYS_PACKET_LENGTH          128
static unsigned char CTPM_FW[]=
{
	#include "hycon_update_fw.i"
};
#endif
#ifdef HYS_APK_DEBUG//HY46XX_FW_URDATE_WITH_BIN_FILE
#define PROC_UPGRADE				0
#define PROC_READ_REGISTER			1
#define PROC_WRITE_REGISTER			2
#define PROC_AUTOCLB				4

#define PROC_NAME	"hy46xx-debug"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *hy46xx_proc_entry;

#endif
/************************************************************/
/*struct hy46xx_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int reset;
};
*/
struct ts_event {
	unsigned short au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	unsigned short au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	unsigned char au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	unsigned char au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	unsigned short pressure;
	unsigned char touch_point;
};
  struct hy46xx_ts_data {
  spinlock_t irq_lock;
  s32 irq_is_disable;
  u8  int_trigger_type;
	//struct work_struct  work;
  struct regulator *tp_regulator;
	unsigned int irq;
  int irq_pin;
  int rst_pin;
  unsigned long irq_flags;
	unsigned int x_max;
	unsigned int y_max;
	u16 abs_x_max;
	u16 abs_y_max;
 	u8  hy46xx_rawdiff_mode;
	u8  hy46xx_is_suspend;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	//struct hy46xx_platform_data *pdata;
	struct  tp_device  tp;
	#ifdef CONFIG_PM
	struct early_suspend *early_suspend;
	#endif
};
#ifdef HYS_APK_DEBUG
typedef struct hy_rw_i2c
{
	unsigned char *buf;
	unsigned char flag;	/*0-write 1-read*/
	unsigned short length;
}*phy_rw_i2c;

typedef struct hy_rw_i2c_queue
{
	struct hy_rw_i2c __user *i2c_queue;
	int queuenum;
}*phy_rw_i2c_queue;

typedef struct hy_rw_i2c1
{
	unsigned char* bytearr_native;
}*phy_rw_i2c1;
typedef struct hy_rw_i2c_queue1
{
	struct hy_rw_i2c1 __user* i2c_queue1;
	int num;
}*phy_rw_i2c_queue1;
/*
static int hy_rw_iic_drv_major = HY_RW_IIC_DRV_MAJOR;
struct hy_rw_i2c_dev {
	struct cdev cdev;
	struct mutex hy_rw_i2c_mutex;
	struct i2c_client *client;
};

struct hy_rw_i2c_dev *hy_rw_i2c_dev_tt;
static struct class *hys_class;
*/
#endif
/************************************************************/
/*
int hy46xx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
int hy46xx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);
void hy46xx_reset_tp();
int hy_rw_iic_drv_init(struct i2c_client *client);
void  hy_rw_iic_drv_exit(void);
int hy46xx_create_sysfs(struct i2c_client * client);
int hy46xx_remove_sysfs(struct i2c_client * client);
int hy46xx_create_apk_debug_channel(struct i2c_client *client);
void hy46xx_release_apk_debug_channel(void);
void  hys_ctpm_fw_upgrade(struct i2c_client * client, unsigned char* pbt_buf, unsigned long dw_lenth, unsigned char lowbyte);
static int hy46xx_read_BootloaderVer(struct i2c_client* client, unsigned char* Boot_Ver);
static int i2c_write_interface(struct i2c_client* client, unsigned char* pbt_buf, int dw_lenth);
static int i2c_read_interface(struct i2c_client* client, unsigned char* pbt_buf, int dw_lenth);
int hy46xx_write_reg(struct i2c_client * client,unsigned char regaddr, unsigned char regvalue);
int hy46xx_read_reg(struct i2c_client * client,unsigned char regaddr, unsigned char * regvalue);
*/
#endif
