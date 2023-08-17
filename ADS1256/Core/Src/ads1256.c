/*
	Author: amir hossein  ershaderad
	Date: 17.July.2023
	library ads1255/1256 for stm32
*/
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <ads1256.h>
#include <dwt_delay.h>
/*
	***************************
	** PART 0 - enumerations **
	***************************
	Enumerations:
		- PGA   - programmable gain amplifier (PGA) settings
		- DRATE - data rate of programmable filter settings
		- REG   - register control adresses
		- CMD   - commands for controlling operation of ADS1256
		- AIN   - input analog channels
		- bool  - boolean True, False
*/
//  Set the Programmable gain amplifier (PGA).
//	PGA Provides more resolution when measuring smaller input signals.
//	Set the PGA to the highest possible setting.

/*
	*******************************
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
// SPICS (ADS1256 chip select) - allows individual selection of a ADS1256 device 
// when multiple devices share the serial bus. 
// Low - for the duration of the serial communication, high - serial interface is reset 
// and DOUT enters high impedance state.
void CS_1(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->csPort,ads->csPin ,GPIO_PIN_SET);
}
void CS_0(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->csPort,ads->csPin ,GPIO_PIN_RESET);
}
// RST (ADS1256 reset output)
void RST_1(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->resetPort,ads->resetPin,GPIO_PIN_SET);
}
void RST_0(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->resetPort,ads->resetPin,GPIO_PIN_RESET);
}
// Delay in microseconds.
void delayus(uint32_t microseconds)
{
	 DWT_Delay(microseconds);
}
// Send 8 bit value over serial interface (SPI).
uint8_t send8bit(uint8_t data,ADS125X_t *ads)
{
  uint16_t size=sizeof(data);
	while(HAL_SPI_Transmit(ads->hspix,&data,size,10)!=HAL_OK);
        return HAL_OK;
}
// Recieve 8 bit value over serial interface (SPI).
uint8_t recieve8bit(uint8_t *read,ADS125X_t *ads)
{
        uint16_t size=sizeof(read);
	while(HAL_SPI_Receive(ads->hspix,read,size,10)!=HAL_OK);
        return HAL_OK;
}
// Wait DRDY (ads1256 data ready output) - used as status signal to indicate when 
// conversion data is ready to be read.  
// Low  - new 24 bits data avaliable, high - 24 bits are read or new data is being updated.
void waitDRDY(ADS125X_t *ads)
{
  while(HAL_GPIO_ReadPin(ads->drdyPort, ads->drdyPin) == GPIO_PIN_SET);
}
/*
	*****************************
	** PART 2 - ads1256 driver **
	*****************************
	Functions:
		- readByteFromReg()
		- writeByteToReg()
		- writeCMD()
                - setBuffer() 
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
		- scanSEChannelContinuous()
		- scanDIFFChannelContinuous()
                - buffer_on()
                - Voltage_Convert_aray()
                - Voltage_Convert()
*/
// Read 1 byte from register address registerID. 
// This could be modified to read any number of bytes from register!
void readByteFromReg(uint8_t registerID,uint8_t *read,uint8_t NM_REG,ADS125X_t *ads)
{
	CS_0(ads);//change to set and reset GPIO PIN
	send8bit(CMD_RREG | registerID,ads); // 1st byte: address of the first register to read
        NM_REG=NM_REG-1;
	send8bit(NM_REG,ads); 				 // 2nd byte: number of bytes to read = 1.
	delayus(7); 	// min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec
	recieve8bit(read,ads);
	CS_1(ads);
}

// Write value (1 byte) to register address registerID.
// This could be modified to write any number of bytes to register!
void writeByteToReg(uint8_t registerID, uint8_t value,uint8_t NM_REG,ADS125X_t *ads)
{
	CS_0(ads);
	send8bit(CMD_WREG | registerID,ads); // 1st byte: address of the first register to write
        NM_REG= NM_REG-1;
	send8bit(NM_REG,ads); 				 // 2nd byte: number of bytes to write = NM_REG.
	send8bit(value,ads);				 // 3rd byte: value to write to register
	CS_1(ads);
}

// Send standalone commands to register.
void writeCMD(uint8_t command,ADS125X_t *ads)
{
	CS_0(ads);
	send8bit(command,ads);
	CS_1(ads);
}

// Set the internal buffer (True - enable, False - disable).nees to change for work 
uint8_t setBuffer(bool val,ADS125X_t *ads)//not ok
{
  if (val==True)
  {
    writeByteToReg(REG_STATUS,1<<1,1,ads);
  }
  else 
  {
    writeByteToReg(REG_STATUS,1<<1,1,ads);
  }
        return HAL_OK;
}
// Get data from STATUS register - chip ID information.
uint8_t readChipID(ADS125X_t *ads)
{
 uint8_t id1;
	waitDRDY(ads);
        readByteFromReg(REG_STATUS,&id1,8,ads);
        return id1=id1>>4;// Only bits 7,6,5,4 are the ones to read (only in REG_STATUS) - return shifted value!
}
// Write to MUX register - set channel to read from in single-ended mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setSEChannel(uint8_t channel,ADS125X_t *ads)
{
	writeByteToReg(REG_MUX, channel << 4 | 1 << 3,8,ads); // xxxx1000 - AINp = channel, AINn = AINCOM
}
// Write to MUX register - set channel to read from in differential mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setDIFFChannel(uint8_t positiveCh, uint8_t negativeCh,ADS125X_t *ads)
{
	writeByteToReg(REG_MUX, positiveCh << 4 | negativeCh,8,ads); // xxxx1000 - AINp = positiveCh, AINn = negativeCh
}

// Write to A/D control register - set programmable gain amplifier (PGA).
// CLKOUT and sensor detect options are turned off in this case.
void setPGA(uint8_t pga,ADS125X_t *ads)//ok
{
  writeByteToReg(REG_ADCON,pga,3,ads);// 00000xxx -> xxx = pga ,sensor Detect current Sources Off,clock out Off
}
// Write to A/D data rate register - set data rate.
void setDataRate(uint8_t drate,ADS125X_t *ads)
{
	writeByteToReg(REG_DRATE, drate,8,ads);
}
//disable and enable Auto Calibration
void auto_calibration(uint8_t value,ADS125X_t *ads)
{
  writeByteToReg(REG_STATUS, value<<2,1,ads);
}
//sensor detect situation 
// need for nuffer on must be turned on before that 
void Sensor_Detect(uint8_t sensor,ADS125X_t *ads)
{
  writeByteToReg(REG_ADCON,sensor<<3,2,ads);
}
// Read 24 bit value from ADS1256. Issue this command after DRDY goes low to read s single
// conversion result. Allows reading data from multiple different channels and in 
// single-ended and differential analog input.
int32_t readData(ADS125X_t *ads)
{
	int32_t read = 0;
	uint8_t buffer[3];

	CS_0(ads);
	send8bit(CMD_RDATA,ads);
	delayus(7); // min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec

        while( HAL_SPI_Receive(ads->hspix, buffer, sizeof(buffer), 10) != HAL_OK ); 
	// DRDY goes high herebuffer

	// construct 24 bit value
	read =  ((uint32_t)buffer[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buffer[1] << 8);
	read |= buffer[2];
	if (read & 0x800000){
		read |= 0xFF000000;
	}
	CS_1(ads);

	return (int32_t)read;
}

// Get one single-ended analog input value by issuing command to input multiplexer.
// It reads a value from previous conversion!
// DRDY needs to be low!
int32_t getOneValSEChannel(uint8_t channel,ADS125X_t *ads)
{
	int32_t read = 0;
	setSEChannel(channel,ads); // MUX command
	delayus(3); // min delay: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC,ads);    // SYNC command
	delayus(3);
	writeCMD(CMD_WAKEUP,ads);  // WAKEUP command
	delayus(1); // min delay: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	read = readData(ads);
	return read;
}

// Get one differential analog input value by issuing command to input multiplexer.
// It reads a value from previous conversion!
// DRDY needs to be low!
int32_t getOneValDIFFChannel(uint8_t positiveCh, uint8_t negativeCh,ADS125X_t *ads)
{
	int32_t read = 0;
	setDIFFChannel(positiveCh, negativeCh,ads);
	delayus(3); // min delayus: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC,ads);
	delayus(3);
	writeCMD(CMD_WAKEUP,ads);
	delayus(1); // min delayus: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	read = readData(ads);
	return  read;
}

// Get one single-ended analog input value from input channels you set (min 1, max 8).
void scanSEChannels(uint8_t channels[], uint8_t numOfChannels, uint32_t *values,ADS125X_t *ads)
{
	for (int i = 0; i < numOfChannels; ++i){
		waitDRDY(ads);
		values[i] = getOneValSEChannel(channels[i],ads);
	}
}

// Get one differential analog input value from input channels you set (min 1, max 4).
void scanDIFFChannels(uint8_t positiveChs[], uint8_t negativeChs[], uint8_t numOfChannels, uint32_t *values,ADS125X_t *ads)
{
	for (int i = 0; i < numOfChannels; ++i){
		waitDRDY(ads);
		values[i] = getOneValDIFFChannel(positiveChs[i], negativeChs[i],ads);
	}
}

// Continuously acquire analog data from one single-ended analog input.
// Allows sampling of one single-ended input channel up to 30,000 SPS.

void scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, int32_t *values, uint32_t *currentTime,ADS125X_t *ads)
{
        uint8_t buffer[3];
	uint32_t read = 0;
        setSEChannel(AIN0,ads);
	delayus(7);
	// Set continuous mode.
	CS_0(ads);
	waitDRDY(ads);
	send8bit(CMD_RDATAC,ads); 
	delayus(7); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds
	// Start reading data
	clock_t startTime = clock();
        for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY(ads);
                while( HAL_SPI_Receive(ads->hspix,buffer, 3, 10) != HAL_OK );
		// construct 24 bit value
                read=(uint32_t)((((buffer[0] & 0x80) ? (0xFF) :(0x00)) <<24)|
                                ((buffer[0] & 0xFF ) << 16)|
                                ((buffer[1] & 0xFF ) << 8 )|
                                ((buffer[2] & 0xFF ) << 0 ));
		values[i] = read;
		buffer[0]=0;
                buffer[1]=0;
                buffer[2]=0;
	}
        currentTime[0] = clock() - startTime;
	// Stop continuous mode.
	waitDRDY(ads);
	send8bit(CMD_SDATAC,ads); // Stop read data continuous.
	CS_1(ads);
}

// Continuously acquire analog data from one differential analog input.
// Allows sampling of one differential input channel up to 30,000 SPS.
void scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, int32_t *values, uint32_t *currentTime,ADS125X_t *ads)
{
	uint8_t buffer[3];
	uint32_t read = 0;
	// Set differential analog input channel.
	setDIFFChannel(positiveCh, negativeCh,ads);
	delayus(7);
	// Set continuous mode.
	CS_0(ads);
	waitDRDY(ads);
	send8bit(CMD_RDATAC,ads);
	delayus(7); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds
	// Start reading data.
	clock_t startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY(ads);
		while( HAL_SPI_Receive(ads->hspix,buffer, 3, 10) != HAL_OK );
		// construct 24 bit value
                read=(uint32_t)((((buffer[0] & 0x80) ? (0xFF) :(0x00)) <<24)|
                                ((buffer[0] & 0xFF ) << 16)|
                                ((buffer[1] & 0xFF ) << 8 )|
                                ((buffer[2] & 0xFF ) << 0 ));
                                  
		values[i] =(int32_t) read;
		buffer[0]=0;
                buffer[1]=0;
                buffer[2]=0;
	}
        currentTime[0] = clock() - startTime;
	// Stop continuous mode.
	waitDRDY(ads);
	send8bit(CMD_SDATAC,ads); // Stop read data continuous.
	CS_1(ads);
}

//for buffer on need a process to turnon buffer 
void buffer_on(ADS125X_t *ads)
{
 //setBuffer(False,ads);
 writeByteToReg(REG_STATUS,0x30,8,ads);
 delayus(20);
 writeCMD(CMD_SELFCAL,ads);
 writeByteToReg(REG_STATUS,0x34,8,ads);
 //delayus(500000);
 HAL_Delay(500);//cant use the delayus(500000) if input of delayus() more 256 this function not worked
 waitDRDY(ads);
 delayus(20);
 //setBuffer(True,ads);
 writeByteToReg(REG_STATUS,0x36,8,ads);
 delayus(20);
}

//Voltage value conversion function
//Vref : The reference voltage 3.3V or 5V
//for time we use the Continuously and we have aray
void  Voltage_Convert_aray(int32_t adc_result[],uint8_t pga,float VREF_VOLTAGE, uint32_t numOfConvert)
{ 
   /* Vin = ( (2*Vr) / G ) * ( x / (2^23 -1)) */ 
    float voltage_uv;
    for(int i=0;i<numOfConvert;i++) 
    {
      voltage_uv = (float)adc_result[i] * VREF_VOLTAGE / 8388607.0 ;
      voltage_uv /= (float)(pga+1);
      voltage_uv *= 1000000;
      adc_result[i]=(int32_t)voltage_uv;
      voltage_uv=0;
    }
}

//Voltage value conversion function
//Vref : The reference voltage 3.3V or 5V
//for time we have only one Variable
int32_t  Voltage_Convert(int32_t adc_result,uint8_t pga,float VREF_VOLTAGE)
{
    /* Vin = ( (2*Vr) / G ) * ( x / (2^23 -1)) */ 
    float voltage_uv = (float)adc_result *2.0 * VREF_VOLTAGE / 8388607.0 ;
    voltage_uv /= (float)(pga+1);
    voltage_uv *= 1000000;
    return (int32_t)voltage_uv;
}