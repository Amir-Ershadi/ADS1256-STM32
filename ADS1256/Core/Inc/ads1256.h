/*
	Author: amir hossein  ershaderad
	Date: 17.July.2023
	library for ads1255/1256 for stm32
*/

#ifndef ADS1256_H_INCLUDED
#define ADS1256_H_INCLUDED

#define STM32F4

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_gpio.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_gpio.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_gpio.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_spi.h"
#include "stm32f7xx_hal_gpio.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_gpio.h"
#endif

#include <stdio.h>
  
  
enum
{
	PGA_GAIN1	= 0, // Input voltage range: +- 5 V
	PGA_GAIN2	= 1, // Input voltage range: +- 2.5 V
	PGA_GAIN4	= 2, // Input voltage range: +- 1.25 V
	PGA_GAIN8	= 3, // Input voltage range: +- 0.625 V
	PGA_GAIN16	= 4, // Input voltage range: +- 0.3125 V
	PGA_GAIN32	= 5, // Input voltage range: +- 0.15625 V
	PGA_GAIN64	= 6  // Input voltage range: +- 0.078125 V
};

//  Set a data rate of a programmable filter (programmable averager).
//	Programmable from 30,000 to 2.5 samples per second (SPS).
//	Setting the data rate to high value results in smaller resolution of the data.
enum
{
	DRATE_30000 = 0xF0, 
	DRATE_15000 = 0xE0,
	DRATE_7500  = 0xD0,
	DRATE_3750  = 0xC0,
	DRATE_2000  = 0xB0,
	DRATE_1000  = 0xA1,
	DRATE_500   = 0x92,
	DRATE_100   = 0x82,
	DRATE_60    = 0x72,
	DRATE_50    = 0x63,
	DRATE_30    = 0x53,
	DRATE_25    = 0x43,
	DRATE_15    = 0x33,
	DRATE_10    = 0x20,
	DRATE_5     = 0x13,
	DRATE_2d5   = 0x03
};

//  Set of registers.
//	The operation of the ADS1256 is controlled through a set of registers. 
//	Collectively, the registers contain all the information needed to configure 
//	data rate, multiplexer settings, PGA setting, calibration, etc.
enum
{
	REG_STATUS = 0,	 // Register adress: 00h, Reset value: x1H
	REG_MUX    = 1,  // Register adress: 01h, Reset value: 01H
	REG_ADCON  = 2,  // Register adress: 02h, Reset value: 20H
	REG_DRATE  = 3,  // Register adress: 03h, Reset value: F0H
	REG_IO     = 4,  // Register adress: 04h, Reset value: E0H
	REG_OFC0   = 5,  // Register adress: 05h, Reset value: xxH
	REG_OFC1   = 6,  // Register adress: 06h, Reset value: xxH
	REG_OFC2   = 7,  // Register adress: 07h, Reset value: xxH
	REG_FSC0   = 8,  // Register adress: 08h, Reset value: xxH
	REG_FSC1   = 9,  // Register adress: 09h, Reset value: xxH
	REG_FSC2   = 10, // Register adress: 0Ah, Reset value: xxH
};


//values for sensor detect 
enum
{
  SENSOR_OFF   =00,
  SENSOR_0_5uA =01,
  SENSOR_2uA   =10,
  SENSOR_10uA  =11,
};
//  This commands control the operation of the ADS1256. 
//	All of the commands are stand-alone except for the register reads and writes 
//	(RREG, WREG) which require a second command byte plus data.
//	CS must stay low (CS_0()) during the entire command sequence.
enum
{
	CMD_WAKEUP   = 0x00, // Completes SYNC and Exits Standby Mode
	CMD_RDATA    = 0x01, // Read Data
	CMD_RDATAC   = 0x03, // Read Data Continuously
	CMD_SDATAC   = 0x0F, // Stop Read Data Continuously
	CMD_RREG     = 0x10, // Read from REG - 1st command byte: 0001rrrr 
						 //					2nd command byte: 0000nnnn
	CMD_WREG     = 0x50, // Write to REG  - 1st command byte: 0001rrrr
						 //					2nd command byte: 0000nnnn
						 // r = starting reg address, n = number of reg addresses
	CMD_SELFCAL  = 0xF0, // Offset and Gain Self-Calibration
	CMD_SELFOCAL = 0xF1, // Offset Self-Calibration
	CMD_SELFGCAL = 0xF2, // Gain Self-Calibration
	CMD_SYSOCAL  = 0xF3, // System Offset Calibration
	CMD_SYSGCAL  = 0xF4, // System Gain Calibration
	CMD_SYNC     = 0xFC, // Synchronize the A/D Conversion
	CMD_STANDBY  = 0xFD, // Begin Standby Mode
	CMD_RESET    = 0xFE, // Reset to Power-Up Values
};

// Input analog channels.
enum
{
	AIN0   = 0, //Binary value: 0000 0000
	AIN1   = 1, //Binary value: 0000 0001
	AIN2   = 2, //Binary value: 0000 0010
	AIN3   = 3, //Binary value: 0000 0011
	AIN4   = 4, //Binary value: 0000 0100
	AIN5   = 5, //Binary value: 0000 0101
	AIN6   = 6, //Binary value: 0000 0110
	AIN7   = 7, //Binary value: 0000 0111
	AINCOM = 8  //Binary value: 0000 1000
};

// Boolean values.
typedef enum
{
	False = 0,
	True  = 1,
} bool;

//struct for data need form stm32 for Functions
// typedef SPI and port,pin for SPI_CS,DRDY,RESET 
typedef struct {
    SPI_HandleTypeDef *hspix;
    GPIO_TypeDef *csPort;
    uint16_t     csPin;
    GPIO_TypeDef *drdyPort;
    uint16_t     drdyPin;
    GPIO_TypeDef *resetPort;
    uint16_t     resetPin;
} ADS125X_t;


// Set custom data types that are 8, 16 and 32 bits long.
#define uint8_t  unsigned char  	// 1 byte
#define uint16_t unsigned short 	// 2 bytes
#define uint32_t unsigned long  	// 4 bytes
//#define uint64_t unsigned long long // 8 bytes

/*	*******************************
	** PART 1 - serial interface **
	*******************************
	Functions:
                - void CS_1();
                - void CS_0();
                - void RST_1();
                - void RST_0();
		- delayus()
		- send8bit()
		- recieve8bit()
		- waitDRDY()

*/
void CS_1(ADS125X_t *ads);//set SPI_CS
void CS_0(ADS125X_t *ads);//reset SPI_CS
void RST_1(ADS125X_t *ads);//set reset_pin
void RST_0(ADS125X_t *ads);//reset reset_pin
void delayus(uint32_t microseconds);// Delay in microseconds.
uint8_t  send8bit(uint8_t data,ADS125X_t *ads);// Send 8 bit value over serial interface (SPI).
uint8_t  recieve8bit(uint8_t *read,ADS125X_t *ads);// Recieve 8 bit value over serial interface (SPI).
void    waitDRDY(ADS125X_t *ads);// Wait until DRDY is low.
/*	*****************************
	** PART 2 - ads1256 driver **
	*****************************
	Functions:
		- readByteFromReg()
		- writeByteToReg()
		- writeCMD()
		- readChipID()
		- setSEChannel()
		- setDIFFChannel()
		- setPGA()
		- setDataRate()
		- readData()
		- getValSEChannel()
		- getValDIFFChannel()
		- scanSEChannels()
		- scanDIFFChannels()
		- scanSEChannelsContinuous()
		- scanDIFFChannelsContinuous()
                - buffer_on()
                - Voltage_Convert_aray()
                - Voltage_Convert()
*/
 
void    readByteFromReg(uint8_t registerID,uint8_t *read,uint8_t NM_REG,ADS125X_t *ads);// Rad 1 byte from register address registerID. 
void    writeByteToReg(uint8_t registerID, uint8_t value,uint8_t NM_REG,ADS125X_t *ads);//Write value (1 byte) to register address registerID.
void    writeCMD(uint8_t command,ADS125X_t *ads);// Send standalone commands to register.
uint8_t setBuffer(bool val,ADS125X_t *ads);// Set the internal buffer (True - enable, False - disable).
uint8_t  readChipID(ADS125X_t *ads);// Get data from STATUS register - chip ID information.
void    setSEChannel(uint8_t channel,ADS125X_t *ads);//Write to MUX register - set channel to read from in single-ended mode.
void    setDIFFChannel(uint8_t positiveCh, uint8_t negativeCh,ADS125X_t *ads);//Write to MUX register - set channel to read from in differential mode.
void    setPGA(uint8_t pga,ADS125X_t *ads);// Write to A/D control register - set programmable gain amplifier (PGA).
void    setDataRate(uint8_t drate,ADS125X_t *ads);// Write to A/D data rate register - set data rate.
void    auto_calibration(uint8_t value,ADS125X_t *ads);//disable and enable Auto Calibration
void    Sensor_Detect(uint8_t sensor,ADS125X_t *ads);//sensor detect situation
int32_t readData(ADS125X_t *ads);// Read 24 bit value from ADS1256. Issue this command after DRDY goes low to read s single
int32_t getOneValSEChannel(uint8_t channel,ADS125X_t *ads);// Get one single-ended analog input value by issuing command to input multiplexer.
int32_t getOneValDIFFChannel(uint8_t positiveCh, uint8_t negativeCh,ADS125X_t *ads);// Get one differential analog input value by issuing command to input multiplexer.
void    scanSEChannels(uint8_t channels[], uint8_t numOfChannels, uint32_t *values,ADS125X_t *ads);// Get one single-ended analog input value from input channels you set (min 1, max 8).
void    scanDIFFChannels(uint8_t positiveChs[], uint8_t negativeChs[], uint8_t numOfChannels, uint32_t *values,ADS125X_t *ads);// Get one differential analog input value from input channels you set (min 1, max 4).
void    scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, int32_t *Values, uint32_t *currentTime,ADS125X_t *ads);// Continuously acquire analog data from one single-ended analog input.
void    scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, int32_t *values, uint32_t *currentTime,ADS125X_t *ads);// Continuously acquire analog data from one differential analog input.
void    buffer_on(ADS125X_t *ads);//for buffer on need a process to turnon buffer
void    Voltage_Convert_aray(int32_t adc_result[],uint8_t pga,float VREF_VOLTAGE, uint32_t numOfConvert);//Voltage value conversion function for time we use the Continuously and we have aray
int32_t Voltage_Convert(int32_t adc_result,uint8_t pga,float VREF_VOLTAGE);//Voltage value conversion function for time we have only one Variable
#endif