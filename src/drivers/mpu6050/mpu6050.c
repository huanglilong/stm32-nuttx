#include "mpu6050.h"
//#include "stm32f4_i2c_mpu6050.h"

static uint8_t buffer[12];          
static int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

static void EXTILine0_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = BSP_INT_ID_EXTI0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)   
{
    I2C_writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range)   
{
    I2C_writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_setFullScaleAccelRange(uint8_t range)  
{
    I2C_writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}


void MPU6050_setSleepEnabled(uint8_t enabled)      
{
    I2C_writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}


uint8_t MPU6050_getDeviceID(void)                  
{
    return I2C_Read(devAddr, MPU6050_RA_WHO_AM_I);
}

uint8_t MPU6050_testConnection(void)              
{
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
     return 1;
   else 
     return 0;
}

void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)    
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_setI2CBypassEnabled(uint8_t enabled)       
{
    I2C_writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_initialize(void) 
{
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); 
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);          // +- 1000dps
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	          // +/- 2g
    MPU6050_setSleepEnabled(0);                         
    MPU6050_setI2CMasterModeEnabled(0);	                

    I2C_writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);     
    I2C_writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);			
    I2C_writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);  
    I2C_writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1); 
    I2C_writeBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);	  

    EXTILine0_Config();
    MPU6050_InitGyro_Offset();
}

unsigned char MPU6050_is_DRY(void)
{
  if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==Bit_SET)
    return 1;  
  else 
    return 0;
}

void MPU6050_getMotion6(MPU6050_Acc_Gyro *data) 
{
      /* Acc data */ 
      buffer[0] = I2C_Read(devAddr, MPU6050_RA_ACCEL_XOUT_H);
      buffer[1] = I2C_Read(devAddr, MPU6050_RA_ACCEL_XOUT_L);
      buffer[2] = I2C_Read(devAddr, MPU6050_RA_ACCEL_YOUT_H);
      buffer[3] = I2C_Read(devAddr, MPU6050_RA_ACCEL_YOUT_L);
      buffer[4] = I2C_Read(devAddr, MPU6050_RA_ACCEL_ZOUT_H);
      buffer[5] = I2C_Read(devAddr, MPU6050_RA_ACCEL_ZOUT_L);
      /* Gyro data */
      buffer[6]  = I2C_Read(devAddr, MPU6050_RA_GYRO_XOUT_H);
      buffer[7]  = I2C_Read(devAddr, MPU6050_RA_GYRO_XOUT_L);
      buffer[8]  = I2C_Read(devAddr, MPU6050_RA_GYRO_YOUT_H);
      buffer[9]  = I2C_Read(devAddr, MPU6050_RA_GYRO_YOUT_L);
      buffer[10] = I2C_Read(devAddr, MPU6050_RA_GYRO_ZOUT_H);
      buffer[11] = I2C_Read(devAddr, MPU6050_RA_GYRO_ZOUT_L);
      /* Calculate Acc and Gyro data */
      data->ax=(((int16_t)buffer[0]) << 8) | buffer[1];
      data->ay=(((int16_t)buffer[2]) << 8) | buffer[3];
      data->az=(((int16_t)buffer[4]) << 8) | buffer[5];
      data->gx=(((int16_t)buffer[6]) << 8) | buffer[7];
      data->gy=(((int16_t)buffer[8]) << 8) | buffer[9];
      data->gz=(((int16_t)buffer[10]) << 8)| buffer[11];
      
      data->ax = (int16_t)(400.0 * data->ax/65535);
      data->ay = (int16_t)(400.0 * data->ay/65535);
      data->az = (int16_t)(400.0 * data->az/65535);
      data->gx = (int16_t)(2000.0*(data->gx - Gx_offset)/65535);
      data->gy = (int16_t)(2000.0*(data->gy - Gy_offset)/65535);
      data->gz = (int16_t)(2000.0*(data->gz - Gz_offset)/65535);
}
void MPU6050_InitGyro_Offset(void)
{
        OS_ERR  err;
	unsigned char i;
	MPU6050_Acc_Gyro temp;
	int32_t	tempgx=0,tempgy=0,tempgz=0;
	int32_t	tempax=0,tempay=0,tempaz=0;
	Gx_offset=0;
	Gy_offset=0;												  
	Gz_offset=0;
	for(i=0;i<10;i++)
	{
  		OSTimeDlyHMSM(0u, 0u, 0u, 1u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
  		MPU6050_getMotion6(&temp);
	}
 	for(i=0;i<100;i++)
	{
		OSTimeDlyHMSM(0u, 0u, 0u, 1u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
		MPU6050_getMotion6(&temp);
		tempax+= temp.ax;
		tempay+= temp.ay;
		tempaz+= temp.az;
		tempgx+= temp.gx;
		tempgy+= temp.gy;
		tempgz+= temp.gz;
	}

	Gx_offset=tempgx/100;//MPU6050_FIFO[3][10];
	Gy_offset=tempgy/100;//MPU6050_FIFO[4][10];
	Gz_offset=tempgz/100;//MPU6050_FIFO[5][10];
}
/****************************************
* fun           : void MPU6050Post(void)
* brief         : for mpu6050 interrupt
* author        : huanglilong
* time          : 2015/8/1
*/
void MPU6050Post(void)
{
  OS_ERR err;
  OS_TCB *MPU6050TCB;
  MPU6050TCB = OSRdyList[APP_CFG_TASK_MPU6050_PRIO].HeadPtr;
  OSTaskSemPost(MPU6050TCB,
                OS_OPT_POST_NONE,
                &err);
}