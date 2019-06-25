#include "bsp_imu.h"


static uint8_t tx, rx;
static uint8_t tx_buff[14];

uint8_t mpu_buff[14];
mpu_data_t mpu_data;

static uint8_t mpu_write_reg(uint8_t const reg, uint8_t const data)
{
	GPIO_ResetBits(GPIOF , GPIO_Pin_6); 	
	tx = reg & 0x7F;
	rx = SPI5_ReadWriteByte(tx);
	tx = data;
	rx = SPI5_ReadWriteByte(tx);
	GPIO_SetBits(GPIOF , GPIO_Pin_6); 	
	return 0;
}

static uint8_t mpu_read_reg(uint8_t const reg)
{
	GPIO_ResetBits(GPIOF , GPIO_Pin_6); 	
	tx = reg | 0x80;
	rx = SPI5_ReadWriteByte(tx);
	rx = SPI5_ReadWriteByte(tx);
	GPIO_SetBits(GPIOF , GPIO_Pin_6); 	
	return 0;
}

static void mpu_offset_cal(void)
{
	int i;
	for(i = 0; i < 500; i++)
	{
		ImuSPI5_ReadData(MPU6500_ACCEL_XOUT_H , mpu_buff , 14);
		
		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

		mpu_data.gx_offset += mpu_buff[8] << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];
		
		delay_ms(1);
	}
	
	mpu_data.ax_offset = mpu_data.ax_offset / 500.0;
	mpu_data.ay_offset = mpu_data.ay_offset / 500.0;
	mpu_data.az_offset = mpu_data.az_offset / 500.0;	
	
	mpu_data.gx_offset = mpu_data.gx_offset / 500.0;
	mpu_data.gy_offset = mpu_data.gy_offset / 500.0;
	mpu_data.gz_offset = mpu_data.gz_offset / 500.0;	
}

static uint8_t MPU_Set_Gyro_LPF(uint8_t data)
{
	//0 250Hz
	//1 184Hz
	//2 92Hz
	//3 41Hz
	//4 20Hz
	//5 10Hz
	//6 5Hz
	//7 3600Hz
  return mpu_write_reg(MPU6500_CONFIG, data);
}

uint8_t mpu_device_init(void)
{
	uint8_t mpu_id;
	
	mpu_write_reg(MPU6500_PWR_MGMT_1, 0x80);
	delay_ms(100);
	
	mpu_write_reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
	delay_ms(100);	
	
	mpu_id = mpu_read_reg(MPU6500_WHO_AM_I);
	
	uint8_t MPU6500_Init_Data[7][2] = {
		{MPU6500_PWR_MGMT_1,           0x03},
		{MPU6500_PWR_MGMT_2,           0x00},
		{MPU6500_CONFIG,               0x04},
		{MPU6500_GYRO_CONFIG,          0x18},		
		{MPU6500_ACCEL_CONFIG,         0x10},		
		{MPU6500_ACCEL_CONFIG_2,       0x04},	
		{MPU6500_USER_CTRL,            0x20},		
	};
	uint8_t i = 0;
	
	for(i = 0 ; i < 7 ; i++)
	{
		mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		delay_ms(1);
	}
	
	mpu_offset_cal();
	
	return 0;
}

void mpu_get_data(void)
{
	float temp;
	ImuSPI5_ReadData(MPU6500_ACCEL_XOUT_H , mpu_buff ,14);
	
	mpu_data.ax    = mpu_buff[0] << 8 | mpu_buff[1];
	mpu_data.ay    = mpu_buff[2] << 8 | mpu_buff[3];
	mpu_data.az    = mpu_buff[4] << 8 | mpu_buff[5];
	mpu_data.temp  = mpu_buff[6] << 8 | mpu_buff[7];
 
	mpu_data.gx    =(mpu_buff[8] << 8 | mpu_buff[9])   - mpu_data.gx_offset;
	mpu_data.gy    =(mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset;
	mpu_data.gz    =(mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset;
}
	









