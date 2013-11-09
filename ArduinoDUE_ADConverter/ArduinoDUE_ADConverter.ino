#include <SPI.h>
#include "AD7193.h"		// AD7193 definitions.
#define AD7193_RDY_STATE        MOSI
#define PMOD1_CS_PIN  10// SS
#define PMOD1_CS_LOW digitalWrite(PMOD1_CS_PIN,LOW);
#define PMOD1_CS_HIGH digitalWrite(PMOD1_CS_PIN,HIGH);
#define AD7193_CH_0      0
unsigned char currentPolarity = 1;
unsigned char currentGain     = 0;

unsigned char AD7193_Init(void)
{
    unsigned char status = 1;
    unsigned char regVal = 0;   
	/*! Initialize SPI communication. */
	SPI.begin(PMOD1_CS_PIN);
	SPI.setDataMode(PMOD1_CS_PIN,SPI_MODE3);
	//SPI.setClockDivider(SPI_CLOCK_DIV4);
        SPI.setClockDivider(PMOD1_CS_PIN,2);// arduino DUE
    AD7193_Reset();
    regVal = AD7193_GetRegisterValue(AD7193_REG_ID, 1, 1);
    if( (regVal & AD7193_ID_MASK)  != ID_AD7193)
    {
        status = 0;
    }	
    return status;
}

void AD7193_SetRegisterValue(unsigned char registerAddress,
                              unsigned long registerValue,
                              unsigned char bytesNumber,
                              unsigned char modifyCS)
{
    unsigned char commandByte = 0;
    unsigned char txBuffer[4] = {0, 0, 0 ,0};
    
    commandByte = AD7193_COMM_WRITE | AD7193_COMM_ADDR(registerAddress);
	
        txBuffer[0] = (registerValue >> 0)  & 0x000000FF;
	txBuffer[1] = (registerValue >> 8)  & 0x000000FF;
	txBuffer[2] = (registerValue >> 16) & 0x000000FF;
	txBuffer[3] = (registerValue >> 24) & 0x000000FF;
	if(modifyCS == 1)
	{
		digitalWrite(PMOD1_CS_PIN, LOW);
	}
	SPI.transfer(PMOD1_CS_PIN,commandByte);
    while(bytesNumber > 0)
    {
        SPI.transfer(PMOD1_CS_PIN, txBuffer[bytesNumber - 1]);
        bytesNumber--;
    }
	if(modifyCS == 1)
	{
		digitalWrite(PMOD1_CS_PIN, HIGH);
	}
}

unsigned long AD7193_GetRegisterValue(unsigned char registerAddress,
                                       unsigned char bytesNumber,
                                       unsigned char modifyCS)

{
    unsigned char receiveBuffer = 0;
    unsigned char writeByte = 0;
    unsigned char byteIndex = 0;
    unsigned long buffer = 0;
    
    writeByte = AD7193_COMM_READ | AD7193_COMM_ADDR(registerAddress);
    if(modifyCS == 1)
	{
		digitalWrite(PMOD1_CS_PIN, LOW);
	}
	SPI.transfer(PMOD1_CS_PIN,writeByte);
    while(byteIndex < bytesNumber)
    {
        receiveBuffer = SPI.transfer(PMOD1_CS_PIN,0);
	buffer = (buffer << 8) + receiveBuffer;
        byteIndex++;
    }
    if(modifyCS == 1)
	{
		digitalWrite(PMOD1_CS_PIN, HIGH);
	}
    return(buffer);

}

void AD7193_SetMode(unsigned char mode)
{
    unsigned long oldRegValue = 0;
    unsigned long newRegValue = 0;

    oldRegValue = AD7193_GetRegisterValue(AD7193_REG_MODE, 3, 1);
    oldRegValue &= ~AD7193_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7193_MODE_SEL(mode);
    AD7193_SetRegisterValue(AD7193_REG_MODE,newRegValue,3,1);                     
}

void AD7193_Reset(void)
{
    char i = 0;
	for(i = 0; i < 6; i++)
	{
		SPI.transfer(PMOD1_CS_PIN,0xFF);
	}    
}

void AD7193_SetPower(unsigned char pwrMode)
{
     unsigned long oldPwrMode = 0x0;
     unsigned long newPwrMode = 0x0; 
 
     oldPwrMode = AD7193_GetRegisterValue(AD7193_REG_MODE, 3, 1);
     oldPwrMode &= ~(AD7193_MODE_SEL(0x7));
     
     newPwrMode = oldPwrMode | 
                  AD7193_MODE_SEL((pwrMode * (AD7193_MODE_IDLE)) |
                                  (!pwrMode * (AD7193_MODE_PWRDN)));

     AD7193_SetRegisterValue(AD7193_REG_MODE, newPwrMode, 3, 1);  

}

void AD7193_WaitRdyGoLow(void)

{
    while(digitalRead(AD7193_RDY_STATE));
}

void AD7193_ChannelSelect(unsigned short channel)

{

    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   

    oldRegValue = AD7193_GetRegisterValue(AD7193_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7193_CONF_CHAN(0x3FF));
    newRegValue = oldRegValue | AD7193_CONF_CHAN(1 << channel);   
    AD7193_SetRegisterValue(AD7193_REG_CONF, newRegValue, 3, 1);

}

void AD7193_Calibrate(unsigned char mode, unsigned char channel)

{

    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    AD7193_ChannelSelect(channel);
    oldRegValue = AD7193_GetRegisterValue(AD7193_REG_MODE, 3, 1);
    oldRegValue &= ~AD7193_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7193_MODE_SEL(mode);
    PMOD1_CS_LOW; 
    AD7193_SetRegisterValue(AD7193_REG_MODE, newRegValue, 3, 0); // CS is not modified.
    AD7193_WaitRdyGoLow();
    PMOD1_CS_HIGH;
}

void AD7193_RangeSetup(unsigned char polarity, unsigned char range)

{

    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;

    oldRegValue = AD7193_GetRegisterValue(AD7193_REG_CONF,3, 1);
    oldRegValue &= ~(AD7193_CONF_UNIPOLAR |
                     AD7193_CONF_GAIN(0x7));

    newRegValue = oldRegValue | 
                  (polarity * AD7193_CONF_UNIPOLAR) |
                  AD7193_CONF_GAIN(range); 

    AD7193_SetRegisterValue(AD7193_REG_CONF, newRegValue, 3, 1);
    /* Store the last settings regarding polarity and gain. */
    currentPolarity = polarity;
    currentGain = 1 << range;

}

unsigned long AD7193_SingleConversion(void)

{
    unsigned long command = 0x0;
    unsigned long regData = 0x0;

    command = AD7193_MODE_SEL(AD7193_MODE_SINGLE) | 
              AD7193_MODE_CLKSRC(AD7193_CLK_INT) |
              AD7193_MODE_RATE(0x060);  
              
    PMOD1_CS_LOW;
    AD7193_SetRegisterValue(AD7193_REG_MODE, command, 3, 0); // CS is not modified.
    AD7193_WaitRdyGoLow();
    regData = AD7193_GetRegisterValue(AD7193_REG_DATA, 3, 0);
    PMOD1_CS_HIGH;

    return regData;

}

unsigned long AD7193_ContinuousReadAvg(unsigned char sampleNumber)

{
    unsigned long samplesAverage = 0;
    unsigned long command        = 0;
    unsigned char count          = 0;
       
    command = AD7193_MODE_SEL(AD7193_MODE_CONT) | 
              AD7193_MODE_CLKSRC(AD7193_CLK_INT) |
              AD7193_MODE_RATE(0x060);
    PMOD1_CS_LOW;
    AD7193_SetRegisterValue(AD7193_REG_MODE, command, 3, 0); // CS is not modified.
    for(count = 0; count < sampleNumber; count++)
    {
        AD7193_WaitRdyGoLow();
        samplesAverage += AD7193_GetRegisterValue(AD7193_REG_DATA, 3, 0); // CS is not modified.
    }
    PMOD1_CS_HIGH;
    samplesAverage = samplesAverage / sampleNumber;
    return samplesAverage;
}
unsigned long AD7193_TemperatureRead(void)
{
    unsigned long dataReg     = 0;
    unsigned long temperature = 0;     
    AD7193_RangeSetup(0, AD7193_CONF_GAIN_1); // Bipolar operation, 0 Gain.
    AD7193_ChannelSelect(AD7193_CH_TEMP);
    dataReg = AD7193_SingleConversion();
    dataReg -= 0x800000;
    temperature = dataReg / 2815;    // Kelvin Temperature
    temperature -= 273;              // Celsius Temperature
    return temperature;
}

float AD7193_ConvertToVolts(unsigned long rawData, float vRef)
{
    float voltage = 0;
    if(currentPolarity == 0 )   // Bipolar mode
    {
        voltage = (((float)rawData / (1ul << 23)) - 1) * vRef / currentGain;
    }
    else                        // Unipolar mode
    {
        voltage = ((float)rawData * vRef) / (1ul << 24) / currentGain;
    }
    return voltage;
}

void setup()
{
   unsigned long regValue = 0;
   Serial.begin(9600);
   delay(7000);
  if(AD7193_Init())
  {
      Serial.print("AD7193 OK");
  }
  else
  {
      Serial.print("AD7193 Error");
  }
  /*! Resets the device. */
    AD7193_Reset();
    Serial.print("\n");
    Serial.print("reset OK");
    Serial.print("\n");
    /*! Select Channel 0 */
    AD7193_ChannelSelect(AD7193_CH_0);
    Serial.print("chanel OK");
    Serial.print("\n");
    /*! Calibrates channel 0. */
    AD7193_Calibrate(AD7193_MODE_CAL_INT_ZERO, AD7193_CH_0);
    Serial.print("calibrate OK");
    Serial.print("\n");
    /*! Selects unipolar operation and ADC's input range to +-2.5V. */
    AD7193_RangeSetup(0, AD7193_CONF_GAIN_1);
    Serial.print("range OK");
    Serial.print("\n");
    regValue = AD7193_GetRegisterValue(AD7193_REG_CONF, 3, 1);// 
    regValue |= AD7193_CONF_PSEUDO;//
    AD7193_SetRegisterValue(AD7193_REG_CONF, regValue, 3, 1);//
    
}
void loop()
{ 
  unsigned long data = 0;
  data = AD7193_SingleConversion();
  float volt=AD7193_ConvertToVolts(data,3.3);
  delay(250);
  Serial.print("volt=");
  Serial.println(volt,DEC);

}
