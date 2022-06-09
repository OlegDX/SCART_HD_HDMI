//#include <iostm8s003f3.h>
//#include "stm8s.h"
#include "stm8s.h"
//#include "stm8s_eval_i2c_ee.h"
//#include "stm8s_eval_lcd.h"
//#include "stm8s_eval.h"
#include "main.h"
#include "Sil9022.h"

#include <string.h>

#define KEY_TSIZE    12

#define RAND_TSIZE   32	// must be a power of two 
#define RAND_TMSK    (RAND_TSIZE - 1)
#define RAND_MULT(x) (x) * 1664525L + 1013904223L
#define RAND_MAX     0x3fff


volatile  uint8_t mem_i2c[0x100] =
{ 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x94, 0x48, 0x44, 0x01, 0x00, 0x00, 0x00
, 0x14, 0x17, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27
, 0x12, 0x48, 0x4C, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C, 0x16, 0x20, 0x58, 0x2C
, 0x25, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x9E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x48
, 0x44, 0x4D, 0x49, 0x20, 0x54, 0x4F, 0x20, 0x53, 0x44, 0x49, 0x0A, 0x20, 0x00, 0x00, 0x00, 0xFD
, 0x00, 0x3B, 0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x7E
, 0x02, 0x03, 0x1E, 0xC1, 0x4A, 0x9F, 0x10, 0x05, 0x14, 0x13, 0x04, 0x07, 0x16, 0x06, 0x15, 0x23
, 0x09, 0x07, 0x07, 0x83, 0x01, 0x00, 0x00, 0x66, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x80, 0x01, 0x1D
, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x1E
, 0xD6, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10, 0x08, 0x60, 0x22, 0x00, 0x12, 0x8E, 0x21, 0x08
, 0x08, 0x18, 0x8C, 0x0A, 0xA0, 0x14, 0x51, 0xF0, 0x16, 0x00, 0x26, 0x7C, 0x43, 0x00, 0x13, 0x8E
, 0x21, 0x00, 0x00, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE
};

#define idx_6F_desc        0x5F
#define idx_18_btn_state   0x08
#define idx_90_session_key 0x80
#define idx_9C_state       0x8C
#define idx_9D             0x8D

#define IDLE_WAITE_I2C   ((uint8_t)0x80)  
#define IDLE_KEY_FALSE   ((uint8_t)0x10)  
#define IDLE_ENABLED     ((uint8_t)0x04)  
#define IDLE_BUTTON      ((uint8_t)0x02)  
#define IDLE_I2C         ((uint8_t)0x01)  

#define SW_I2C1_SCL_GPIO  GPIOC
#define SW_I2C1_SDA_GPIO  GPIOC
#define SW_I2C1_SCL_PIN   GPIO_PIN_4
#define SW_I2C1_SDA_PIN   GPIO_PIN_5
#define READ_CMD          1
#define WRITE_CMD         0
#define SW_I2C1_ADDR      0x14


//uint8_t  p0018_btn_state;
//uint8_t  p0090[12] = {0x02, 0x03, 0x1E, 0xC1, 0x4A, 0x9F, 0x10, 0x05, 0x14, 0x13, 0x04, 0x07};
//uint16_t p009C_state;
//uint8_t  p009D[ 8] = {0x06, 0x15, 0x23, 0x09, 0x07, 0x07, 0x83, 0x01};

volatile uint8_t  p0110_first_key[KEY_TSIZE] = {0x41, 0xFE, 0x7F, 0xF9, 0xDE, 0xE7, 0x6D, 0xF0, 0x72, 0xB9, 0xAE, 0x87};
volatile uint32_t p011C_randSeed = 1;       // seed  Random
volatile uint8_t  p0120_key_update = 50;
volatile uint8_t  p0121_key_update_result = 2;
volatile BitStatus p0122_PC_5_state = SET;
volatile BitStatus p0123_PD_3_state = SET;
volatile BitStatus p0124_PD_2_state = SET;
//--------------------
volatile uint32_t p0125_randVal[RAND_TSIZE];       // prv    Random
volatile uint8_t  p01A5_master_key[KEY_TSIZE];
volatile uint8_t  p01B1_session_key[KEY_TSIZE];
volatile uint8_t  p01BD_extented_key[8];
volatile uint32_t p01C5_randIdx;             // idx    Random
volatile uint16_t p01C9_I2C_Mem_idx;
volatile uint16_t p01CB_PC_5_count;
volatile uint16_t p01CD_PD_3_count;
volatile uint16_t p01CF_PD_2_count;
void   (*p01D1_Random_func)();
void   (*p01D3_Random_func)();
void   (*p01D5_func)();       //Random
void   (*p01D7_func)();       //Random
volatile uint8_t  p01D9;
volatile uint8_t  p01DA;
volatile uint8_t  p01DB;
volatile uint8_t  p01DC;
volatile uint8_t  p01DD_I2C_enable;
volatile uint8_t  p01DE_state;
volatile uint8_t  p01DF_i2c_write_result;
volatile uint8_t  p01E0_I2C_ReceiveData;
volatile uint8_t  p01E1_I2C_Receive_state;
volatile uint8_t  p01E2;
volatile uint8_t  p01E3_randInit; //Random

__no_init volatile uint8_t  p4000 @0x4000;
__no_init volatile uint8_t  p4260[KEY_TSIZE] @0x4260;
__no_init volatile uint8_t  p4270[KEY_TSIZE] @0x4270;
__no_init volatile uint8_t  p427F @0x427F;

//80C4
void DELAY(uint16_t value)
{
  while (value-- > 0)
    ;
}
//8658
void Flash_Unlock_Mem(void)
{
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG);
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
}
//8648
void Flash_Write(uint8_t *data, uint8_t value)
{
  Flash_Unlock_Mem();
  *data = value;
  while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET); //FLASH_IAPSR_EOP
}


/*
#define RAND_TSIZE		32	// must be a power of two 
#define RAND_TMSK		(RAND_TSIZE - 1)
#define RAND_MULT(x)	(x) * 1664525L + 1013904223L
#define RAND_MAX		0x3fff

static char randInit = 0;
static unsigned long randSeed = 1;
static unsigned long randIdx = 0;
static unsigned long randVal[RAND_TSIZE];
static xSemaphoreHandle randMutex;

static int rand( void )
{	
	int j;
	
	
	xSemaphoreTake( randMutex, portMAX_DELAY );
	
	if (randInit == 0)
	{
		for (j = 0; j < 8; ++j)
		{
			randSeed = RAND_MULT(randSeed);
		}
		
		for (j = 0; j < RAND_TSIZE; ++j)
		{
			randVal[j] = (randSeed = RAND_MULT(randSeed));
		}
		
		randIdx = randVal[RAND_TSIZE - 1];
		randInit = 1;
	}
	
	randSeed = RAND_MULT(randSeed);
	j = randIdx & RAND_TMSK;
	randIdx = randVal[j];
	randVal[j] = randSeed;
	
	xSemaphoreGive( randMutex );

	return (randIdx & RAND_MAX);
}
*/

//void    _8ABE_prng_011C(void)
//{
//  p011C_randSeed = (p011C_randSeed * 0x0019660dL + 0x3c6ef35fL) & 0xffffffffL;
//}

//8B03
void Random_Init(uint16_t value) 
{
  if (p01D1_Random_func)
    p01D1_Random_func();

  p01E3_randInit = 0;

  if (p01D3_Random_func)
    p01D3_Random_func();

  p011C_randSeed = value;
}
//8790
uint16_t Random_16(void)
{
  if (p01D1_Random_func)
    p01D1_Random_func();
  if (p01D3_Random_func)
    p01D3_Random_func();
  if (p01D5_func)
    p01D5_func();
  if (p01D7_func)
    p01D7_func();

  if (p01E3_randInit == 0)
  {
    for (int i = 8; i > 0; i--)
    {
      p011C_randSeed = RAND_MULT(p011C_randSeed);
    }
    for (int i = 0; i < RAND_TSIZE; i++)
    {
      p011C_randSeed = RAND_MULT(p011C_randSeed);
      p0125_randVal[i] = p011C_randSeed;
    }
    p01C5_randIdx = p0125_randVal[RAND_TSIZE-1];
    p01E3_randInit = 1;
  }

  p011C_randSeed = RAND_MULT(p011C_randSeed);

  uint8_t l_index = (uint8_t)p01C5_randIdx & RAND_TMSK;
  p01C5_randIdx = p0125_randVal[l_index];
  p0125_randVal[l_index] = p011C_randSeed;

  return ((uint16_t)(p01C5_randIdx >> 16)) & RAND_MAX;
}
//86B9
uint8_t Random_8(void)
{
  return (uint8_t)(Random_16() % 0xff);
}



// void    _85E4_I2C_STOP(void)
// {
//   I2C_GenerateSTOP(ENABLE);
//   I2C_GenerateSTOP(DISABLE);
//   I2C_ClearFlag((I2C_Flag_TypeDef)~I2C_SR2_RESET_VALUE);
//   //I2C->CR2 &= ~I2C_CR2_STOP;  // Disable the STOP condition generation
//   //I2C->CR2 |=  I2C_CR2_STOP;  // Generate a STOP condition
//   //I2C->SR2  = 0;					    // clear all error flags
// }
//void _85D9_I2C_inc_01C9(void)
//{
//  p01C9_I2C_Mem_idx++;
//  p01C9_I2C_Mem_idx &= 0xFF;
//}

void    _8554_I2C_Slave_check_event(void)
{
  uint8_t l_SR1 = I2C->SR1; //clearing status registers
  uint8_t l_SR2 = I2C->SR2; //clearing status registers
  uint8_t l_SR3 = I2C->SR3; //clearing status registers

  if (l_SR1 & I2C_SR1_ADDR) //(ADDR) Our Slave Address has matched
    p01E1_I2C_Receive_state = 0;              //reset the counter after every restart

  //Byte received
  if (l_SR1 & I2C_SR1_RXNE)  //(RXNE) Data Register Not empty
  {
    p01E0_I2C_ReceiveData = I2C_ReceiveData();
    if (p01E1_I2C_Receive_state == 0)
    {
      //ram_address
      p01E1_I2C_Receive_state = 1;
      p01C9_I2C_Mem_idx = p01E0_I2C_ReceiveData;

      if (p01DE_state & IDLE_WAITE_I2C) 
        p01DE_state &= ~IDLE_WAITE_I2C;
    } else {
      //data
      p01E1_I2C_Receive_state = 2;

      if (p01C9_I2C_Mem_idx >= 8)
      {
        uint8_t tmp = p01E0_I2C_ReceiveData;
        mem_i2c[p01C9_I2C_Mem_idx] = tmp;
      }

      p01C9_I2C_Mem_idx++;
      p01C9_I2C_Mem_idx &= 0xFF;
    }
  }

  if (l_SR1 & I2C_SR1_TXE)
  {
    I2C_SendData(mem_i2c[p01C9_I2C_Mem_idx]);
    p01C9_I2C_Mem_idx++;
    p01C9_I2C_Mem_idx &= 0xFF;
  }

  // Communication error?
  if (l_SR2 & (I2C_SR2_WUFH | I2C_SR2_OVR | I2C_SR2_ARLO | I2C_SR2_BERR))
  {
    I2C_GenerateSTOP(ENABLE);
    I2C_GenerateSTOP(DISABLE);
    I2C_ClearFlag((I2C_Flag_TypeDef)~I2C_SR2_RESET_VALUE);
  }

  if (l_SR1 & I2C_SR1_STOPF)
  {
    I2C_GenerateSTOP(ENABLE);
    I2C_GenerateSTOP(DISABLE);
    I2C_ClearFlag((I2C_Flag_TypeDef)~I2C_SR2_RESET_VALUE);
  }
}



uint8_t _898F_PC_5_Idle(void)
{
  BitStatus l_pin = GPIO_ReadInputPin(GPIOC, GPIO_PIN_5);
  if (l_pin == RESET)
  {
    if (p01CB_PC_5_count < 300)
    {
      p01CB_PC_5_count++;
      return 0;
    }
  }
  else
  {
    p01CB_PC_5_count = 0;
  }

  if (l_pin != p0122_PC_5_state)
  {
    if (l_pin == RESET)
    {
      if (p0122_PC_5_state != RESET)
        return 1;
    }
    p0122_PC_5_state = l_pin;
  }
  return 0;
}

uint8_t _89CD_PD_3_Idle(void)
{
  BitStatus l_pin = GPIO_ReadInputPin(GPIOD, GPIO_PIN_3);
  if (l_pin == RESET)
  {
    if (p01CD_PD_3_count > 300)
    {
      p01CD_PD_3_count++;
      return 0;
    }
  }
  else
  {
    p01CD_PD_3_count = 0;
  }

  if (l_pin != p0123_PD_3_state)
  {
    if (l_pin == RESET)
    {
      if (p0123_PD_3_state != RESET)
        return 1;
    }
    p0123_PD_3_state = l_pin;
  }
  return 0;
}

uint8_t _8A09_PD_2_Idle(void)
{
  BitStatus l_pin = GPIO_ReadInputPin(GPIOD, GPIO_PIN_2);
  if (l_pin == RESET)
  {
    if (p01CF_PD_2_count > 300)
    {
      p01CF_PD_2_count++;
      return 0;
    }
  }
  else
  {
    p01CF_PD_2_count = 0;
  }

  if (l_pin != p0124_PD_2_state)
  {
    if (l_pin == RESET)
    {
      if (p0124_PD_2_state != RESET)
        return 1;
    }
    p0124_PD_2_state = l_pin;
  }
  return 0;
}

void    _8BB1_SetButtonState(uint8_t state)
{
  mem_i2c[idx_18_btn_state] |= state;
}

void    _8AE1_Idle_Button()
{
  if (_898F_PC_5_Idle() == 1)
    _8BB1_SetButtonState(0x01);

  if (_89CD_PD_3_Idle() == 1)
    _8BB1_SetButtonState(0x02);

  if (_8A09_PD_2_Idle() == 1)
    _8BB1_SetButtonState(0x04);
}



void    _8609_GPIO_Setup()
{
  GPIOD->DDR &= ~GPIO_PIN_3; // Input mode
  GPIOD->CR1 |= GPIO_PIN_3;   // Pull-Up or Push-Pull

  GPIOD->DDR &= ~GPIO_PIN_2; // Input mode
  GPIOD->CR1 |= GPIO_PIN_2;   // Pull-Up or Push-Pull

  GPIOC->DDR &= ~GPIO_PIN_5; // Input mode
  GPIOC->CR1 |= GPIO_PIN_5;   // Pull-Up or Push-Pull

  GPIOC->DDR |= GPIO_PIN_5; // Output mode !!!!!!ToDo!!!!
  GPIOC->CR1 |= GPIO_PIN_3; // Pull-Up or Push-Pull
  GPIOC->ODR |= GPIO_PIN_5; // High level
}

void    _82F3_SET_4270(uint8_t *data)
{
  uint16_t j = 65000;
  for (uint8_t i = 0; i < 15; i++)
  {
    while(j != 0)
      j--;
    j--;
  }
  Flash_Unlock_Mem();
  for (uint8_t i = 0; i < KEY_TSIZE; i++)
    p4270[i] = data[i];
  p427F = 0;
  //while (FLASH->IAPSR & FLASH_IAPSR_EOP)
  while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET);
}

#if 1

#define I2C1_SDA_OUT()     SW_I2C1_SDA_GPIO->DDR |=  SW_I2C1_SDA_PIN
#define I2C1_SDA_IN()      SW_I2C1_SDA_GPIO->DDR &= ~SW_I2C1_SDA_PIN
#define I2C1_SDA_ReadVal() SW_I2C1_SDA_GPIO->IDR &   SW_I2C1_SDA_PIN ? 1 : 0

#define I2C1_SCL_OUT()     SW_I2C1_SCL_GPIO->DDR |=  SW_I2C1_SCL_PIN
#define I2C1_SCL_IN()      SW_I2C1_SCL_GPIO->DDR &= ~SW_I2C1_SCL_PIN
#define I2C1_SCL_ReadVal() SW_I2C1_SCL_GPIO->IDR &   SW_I2C1_SCL_PIN ? 1 : 0

#define I2C1_SDA_HIGH()    I2C1_SDA_OUT(); SW_I2C1_SDA_GPIO->ODR |=  SW_I2C1_SDA_PIN
#define I2C1_SDA_LOW()     I2C1_SDA_OUT(); SW_I2C1_SDA_GPIO->ODR &= ~SW_I2C1_SDA_PIN

#define I2C1_SCL_HIGH()    I2C1_SCL_OUT(); SW_I2C1_SCL_GPIO->ODR |=  SW_I2C1_SCL_PIN
#define I2C1_SCL_LOW()     I2C1_SCL_OUT(); SW_I2C1_SCL_GPIO->ODR &= ~SW_I2C1_SCL_PIN

#else

//8086
void I2C1_SDA_OUT(void)
{
  SW_I2C1_SDA_GPIO->DDR |= SW_I2C1_SDA_PIN;
}
//81DB
void I2C1_SDA_IN(void)
{
  SW_I2C1_SDA_GPIO->DDR &= ~SW_I2C1_SDA_PIN;
}

//808F
void I2C1_SCL_OUT(void)
{
  SW_I2C1_SCL_GPIO->DDR |= SW_I2C1_SCL_PIN;
}
//8B77 ????
void I2C1_SCL_IN(void)
{
  SW_I2C1_SCL_GPIO->DDR &= ~SW_I2C1_SCL_PIN;
}
//8080
void I2C1_SDA_HIGH(void)
{
  I2C1_SDA_OUT();
  SW_I2C1_SDA_GPIO->ODR |= SW_I2C1_SDA_PIN;
}
//8099
void I2C1_SDA_LOW(void)
{
  I2C1_SDA_OUT();
  SW_I2C1_SDA_GPIO->ODR &= ~SW_I2C1_SDA_PIN;
}
//808F
void I2C1_SCL_HIGH(void)
{
  I2C1_SCL_OUT();
  SW_I2C1_SCL_GPIO->ODR |= SW_I2C1_SCL_PIN;
}
//80A5
void I2C1_SCL_LOW(void)
{
  I2C1_SCL_OUT();
  SW_I2C1_SCL_GPIO->ODR &= ~SW_I2C1_SCL_PIN;
}

//818E ????
uint8_t I2C1_SDA_ReadVal(void)
{
  //I2C1_SDA_IN();
  return SW_I2C1_SDA_GPIO->IDR & SW_I2C1_SDA_PIN ? 1 : 0;
}
//8B77
uint8_t I2C1_SCL_ReadVal(void)
{
  //I2C1_SCL_IN();
  return SW_I2C1_SCL_GPIO->IDR & SW_I2C1_SCL_PIN ? 1 : 0;
}
#endif

//80CD
uint8_t SW_I2C_SCL_WAIT_HIGH(void)
{
  I2C1_SCL_HIGH();
  I2C1_SCL_IN();
  for (uint8_t i = 0; i < 100 ; i++)
  {
    if (I2C1_SCL_ReadVal() == 1)
      return 1;
  }
  return 0;
}
//8175
uint8_t SW_I2C_Read_Data(uint8_t value)
{
  uint8_t readdata = 0;

  I2C1_SDA_IN();
  DELAY(1);

  for (uint8_t i = 0; i < 8; i++)
  {
    SW_I2C_SCL_WAIT_HIGH();
    readdata <<= 1;
    if (I2C1_SDA_ReadVal())
      readdata |= 1;
    I2C1_SCL_LOW();
  }
  DELAY(1);
  
  if (value == 0){
    I2C1_SDA_HIGH();
  }else{
    I2C1_SDA_LOW();
  }
  SW_I2C_SCL_WAIT_HIGH();
  DELAY(1);
  I2C1_SCL_LOW();
  DELAY(1);
  return readdata;
}
//80EE
uint8_t SW_I2C_Write_Data(uint8_t value)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (value & 0x80){
      I2C1_SDA_HIGH();
    }else{
      I2C1_SDA_LOW();
    }
    SW_I2C_SCL_WAIT_HIGH();
    I2C1_SCL_LOW();
    value <<= 1;
  }
  I2C1_SDA_IN();
  DELAY(1);
  SW_I2C_SCL_WAIT_HIGH();
  DELAY(1);

  uint8_t ret = I2C1_SDA_ReadVal();
  p01DF_i2c_write_result = ret;

  I2C1_SCL_LOW();
  return ret;
}
//8145
void SW_I2C_Start(uint8_t mode, uint8_t addr)
{
  I2C1_SDA_HIGH();
  DELAY(12);
  I2C1_SCL_HIGH();
  DELAY(12);
  I2C1_SDA_LOW();
  DELAY(12);
  I2C1_SCL_LOW();
  //slave address
  SW_I2C_Write_Data(addr | mode);
}
//81E8
void SW_I2C_Stop(void)
{
  I2C1_SDA_LOW();
  SW_I2C_SCL_WAIT_HIGH();
  I2C1_SDA_HIGH();
  
  I2C1_SCL_IN();
  I2C1_SDA_IN();
}
//81FD
uint8_t SW_I2C_ReadFromValue(uint8_t value, uint8_t addr)
{
  SW_I2C_Start(WRITE_CMD, addr);
  SW_I2C_Write_Data(value);
  
  SW_I2C_Start(READ_CMD, addr);
  uint8_t ret = SW_I2C_Read_Data(1);
  SW_I2C_Stop();
  return ret;
}

//80B3
//void PC_Set_InputMode_PIN4_PIN5(void)
//{
//  I2C1_SCL_IN();
//  I2C1_SDA_IN();
//}
//82DE
//void _82DE_0_14(void)
//{
//  SW_I2C_Start(0, 0x14);
//}
//81F7
//void _81F7_(uint8_t value)
//{
//  SW_I2C_Write_Data(value);
//  SW_I2C_Stop();
//}

uint8_t _8227_update_0121(void)
{
  uint8_t summ = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    p01BD_extented_key[i] = Random_8();
    summ += p01BD_extented_key[i];
    SW_I2C_Start(WRITE_CMD, SW_I2C1_ADDR);
    SW_I2C_Write_Data(i);
    uint8_t ret = SW_I2C_Write_Data(p01BD_extented_key[i]);
    SW_I2C_Stop();
    if (ret != 0)
    {
      p0121_key_update_result = 1;
      return 1;
    }
  }

  //825E
  SW_I2C_Start(WRITE_CMD, SW_I2C1_ADDR);
  SW_I2C_Write_Data(0x10);
  SW_I2C_Write_Data(0x01);
  SW_I2C_Stop();
  DELAY(100);
  
  //8271
  for (uint8_t i = 0; i < 7; i++)
  {
    p01BD_extented_key[i] = (summ + p01BD_extented_key[i+1]) ^ 0xFF;
  }
  //8288
  p01BD_extented_key[7] = summ;

  DELAY(100);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (SW_I2C_ReadFromValue(i, SW_I2C1_ADDR) != p01BD_extented_key[i])
    {
      p0121_key_update_result = 2;
      return 2;
    }
    if (p01DF_i2c_write_result != 0)
    {
      p0121_key_update_result = 1;
      return 1;
    }
  }
  
  //82BA
  if (SW_I2C_ReadFromValue(8, SW_I2C1_ADDR) + summ == 0xFF)
  {
    p0121_key_update_result = 2;
    return 2;
  }
  p0121_key_update_result = 0;
  return 0;
}
//8339 +
void DecodeKey(void)
{
  uint8_t buff[12] = {0};
  
  //  p0110_first_key[ 0] = 'a';
  //  p0110_first_key[ 1] = 'n';
  //  p0110_first_key[ 2] = 'q';
  //  p0110_first_key[ 3] = 'i';
  //  p0110_first_key[ 4] = 'n';
  //  p0110_first_key[ 5] = 'g';
  //  p0110_first_key[ 6] = 'a';
  //  p0110_first_key[ 7] = 'n';
  //  p0110_first_key[ 8] = 'q';
  //  p0110_first_key[ 9] = 'i';
  //  p0110_first_key[10] = 'n';
  //  p0110_first_key[11] = 'g';
  memcpy((uint8_t*)p0110_first_key, "anqinganqing", 12);

  p01D9 = 0x40;
  p01DA = 1;
  uint8_t a8 = 0;
  while (1)
  {
    for (uint8_t i = 0; i < 12; i++)
    {
      //8411
      uint16_t a4 = (uint16_t)p4260[i];
      a4 += (uint16_t)p0110_first_key[i];

      uint8_t a7 = i % 3;
      if (a7 == 0)
        a4 = ~((~a4) | 0x40);
      else if (a7 == 1)
        a4 |= 0x0A;
      else if (a7 == 2)
        a4 = (a4 << 1) | 0x10;
      
      //83B2
      buff[i] = (uint8_t)a4;
      p01A5_master_key[i] = (uint8_t)a4;
    }

    p01DB = 0x41;
    //p01DB = p01DA | p01D9;
    if (p427F == 0x41)
    {
      uint8_t l_ret = 3;
      while (l_ret > 2)
        l_ret =_8227_update_0121();

      if (l_ret == 0)
      {
        if (p4270[11] == 0)
        {
          _82F3_SET_4270(&buff[0]);
          continue;
        }
        a8 = 1;
      }
      //83E8
      else if (l_ret == 1)
      {
        if (p0120_key_update != 0)
          continue;
        a8 = 1;
      }
      //83EC
      else if (l_ret == 2)
      {
        a8 = 1;
      }
    }
    // 8453
    else if (p427F == 0)
    {
      for (uint8_t i = 0; i < 12; i++)
      {
        if (p4270[i] != buff[i])
        {
          a8 = 1;
          break;
        }
      }
    }
    else
    {
      a8 = 1;
    }
    break;
  }
  //_847D
  p01DE_state = IDLE_WAITE_I2C | IDLE_I2C | (a8 * IDLE_KEY_FALSE);
  while (a8)
    ;
}

//8673
void Random_Setup(void)
{
  Random_Init(p4000);

  for (int i = 0; i < KEY_TSIZE; i++)
  {
    mem_i2c[idx_90_session_key+i] = Random_8();
    p01B1_session_key[i] = mem_i2c[idx_90_session_key+i];
  }

  Flash_Write((uint8_t*)&p4000, mem_i2c[idx_90_session_key]);
}
//8516
void I2C_Setup(void)
{
  // I2C_CR1 |= I2C_CR1_PE;                    // Enable I2C
  // I2C_CR2 |= I2C_CR2_ACK;                   // I2C_AcknowledgeConfig(I2C_ACK_CURR)
  // I2C_FREQR &= (uint8_t)(~I2C_FREQR_FREQ);  // Peripheral Clock Frequency -Clear frequency bits
  // I2C_FREQR |= 0x01;                        // InputClockFrequencyMHz
  // I2C_OARL = 0x40;                          // I2C_OARH_ADDCONF = 0x40  //Address Mode Configuration
  // I2C_OARH = 0x40;
  // I2C_ITR |= I2C_ITR_ITEVTEN; //Event Interrupt Enable   I2C_IT_EVT
  // I2C_ITR |= I2C_ITR_ITBUFEN; //Buffer Interrupt Enable   I2C_IT_BUF

  /* I2C Initialize */
  I2C_Init(I2C_MAX_STANDARD_FREQ, 0x40, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 1);
  /* Enable Buffer and Event Interrupt*/
  I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_EVT | I2C_IT_BUF), ENABLE);

  rim();
}
//8495
uint8_t CompareKey(uint8_t *value)
{
  p01DC = p01DB;
  for (uint8_t i = 0; i < KEY_TSIZE; i++)
  {
    if (p4270[i] != value[i] || value[i] == 0)
    {
        p01DC = 0;
        return 0;
    }
  }
  return p01DB;
}
//86C9
void _86C9_main_init(void)
{
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  DecodeKey();
  _8609_GPIO_Setup();
  Random_Setup();
  I2C_Setup();
  // enableInterrupts();
  rim();
}
//84C6
void _84C6_main_init(void)
{
  if ((p0110_first_key[0] - 32) == CompareKey((uint8_t*)p01A5_master_key))
    p01DD_I2C_enable = p0110_first_key[1];

  p01DE_state |= IDLE_BUTTON;
}
//88B7
void _88B7_update_009D(void)
{
  uint8_t sum = mem_i2c[idx_9D+1];
    sum += mem_i2c[idx_9D+0];
    sum += mem_i2c[idx_9D+2];
    sum += mem_i2c[idx_9D+3];
  uint8_t val = p01B1_session_key[sum % KEY_TSIZE];
  mem_i2c[idx_9D+4] =   sum + mem_i2c[idx_9D + 0] + val;
  mem_i2c[idx_9D+5] = ((sum + mem_i2c[idx_9D + 1]) & val) << 2;
  mem_i2c[idx_9D+6] =  (sum + mem_i2c[idx_9D + 2] + val) | 0x56;
  mem_i2c[idx_9D+7] = ((sum + mem_i2c[idx_9D + 3]) | val) ^ 0xFF;
}

void _8A70_idle_I2C(void)
{
  if ((mem_i2c[idx_9C_state] & 0x80) != 0)
  {
    p01E2 = 0;
    mem_i2c[idx_9C_state] &= ~0x80;
  }

  if ((mem_i2c[idx_9C_state] & 0x02) != 0)
  {
    _88B7_update_009D();
    p01DE_state &= ~IDLE_ENABLED;
    mem_i2c[idx_9C_state] = 0x01;
  }
}
//84F1 +
void _84F1_main_idle(void)
{
  uint8_t tmp = p01DD_I2C_enable;
  if (p0110_first_key[4] != tmp)
  {
    I2C_Cmd(DISABLE);
    p01DE_state = 0;
  }
  else
  {
    if ((p01DE_state & IDLE_ENABLED) == 0)
      p01DE_state |= IDLE_ENABLED;
  }
}

int main(void)
{
  _86C9_main_init();
  _84C6_main_init();
  while (1)
  {
    if ((p01DE_state & ~IDLE_I2C) == (IDLE_ENABLED | IDLE_BUTTON))
      _8AE1_Idle_Button();

    if ((p01DE_state & ~IDLE_BUTTON) == (IDLE_ENABLED | IDLE_I2C))
      _8A70_idle_I2C();

    _84F1_main_idle();
  }
}


//0x405d 43 42 7B 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4A 33 00 34 00 00 00 00
//0x40dd FF
//0x4260 00 2B 00 06 0E 47 35 33 31 38 37 33 00 00 00 00
//0x4270 DE 9B 38 D0 7E 57 69 AB 51 5E AF 5D 00 00 00 00

/*



int main_(void)
{
#ifdef FAST_I2C_MODE
  // system_clock / 1 
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
#else
  // system_clock / 2 
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV2);
#endif

  // I2C Initialize 
  //I2C_Init(I2C_SPEED, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);

  // Enable Buffer and Event Interrupt
  I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_EVT | I2C_IT_BUF), ENABLE);

  enableInterrupts();

  // TXBuffer initialization 
  //for (i = 0; i < BUFFERSIZE; i++)
  //  TxBuffer[i] = i;

  // Data Direction Register
  // 0: Input
  // 1: Output
  // PD_DDR_bit.DDR0 = 1;

  // Control Register 1
  // Input mode:
  //   0: Floating input
  //   1: Input with pull-up
  // Output mode:
  //   0: Pseudo open drain
  //   1: Push-pull
  // PD_CR1_bit.C10 = 1;

  // Control Register 2
  // Input mode:
  //   0: External interrupt disabled
  //   1: External interrupt enabled
  // Output mode:
  //   0: Output speed up to  2 MHz
  //   1: Output speed up to 10 MHz
  // PD_CR2_bit.C20 = 1;

  // Output Data Register
  // Output value
  // PD_ODR_bit.ODR0 = 0;

  // Main loop
  while (1)
  {
    // PD_ODR_bit.ODR0 = !PD_ODR_bit.ODR0;
    //delay(60000);
  }
  // return 0;
}
*/

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval : None
 */
void assert_failed(u8 *file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif