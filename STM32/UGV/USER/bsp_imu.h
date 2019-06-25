#include "mpu6500_reg.h"
#include "spi.h"
#include "delay.h"


typedef struct
{
  int16_t ax;
	int16_t ay;
	int16_t az;
	
	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t temp;
	
	int16_t mx;
	int16_t my;
	int16_t mz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;

}mpu_data_t;

extern mpu_data_t mpu_data;


uint8_t mpu_device_init(void);
void mpu_get_data(void);









