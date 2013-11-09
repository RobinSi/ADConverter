#include "AD7193.h"		// AD7193 definitions.
#define AD7193_RDY_STATE        MOSI //define pin MOSI( MISO is automaticaly defined)
#define PMOD1_CS_PIN            SS // define the chipselect
#define PMOD1_CS_LOW digitalWrite(SS,LOW); // reset chipselect 
#define PMOD1_CS_HIGH digitalWrite(SS,HIGH);//set chipselect 

/*define all channel in PmodAD5 for pseudo-differential using*/
#define AD7193_CH_0      0
#define AD7193_CH_1      1
#define AD7193_CH_2      2
#define AD7193_CH_3      3
#define AD7193_CH_4      4
#define AD7193_CH_5      5
#define AD7193_CH_6      6
#define AD7193_CH_7      7
#define AD7193_CH_8      8

unsigned char currentPolarity = 1;// unipolar mode
unsigned char currentGain     = 0;


/*! Checks if the AD7139 part is present. */
unsigned char AD7193_Init(void)

{//AD7193_Init
    unsigned char status = 1;
    unsigned char regVal = 0;
   
    status = SPI_Init(0,2,1, 0);// initialize SPI on Cerebot Mx3cK
    regVal = AD7193_GetRegisterValue(AD7193_REG_ID, 1, 1);

    if((regVal & AD7193_ID_MASK) != ID_AD7193)
    {
        status = 0;
    }
    return status;

}//AD7193_Init

/*! Writes data into a register. */
void AD7193_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber,
                             unsigned char modifyCS)

{//setregistervalue
    unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;

    writeCommand[0] = AD7193_COMM_WRITE |
                      AD7193_COMM_ADDR(registerAddress);
                      
    while(bytesNr > 0)
    {
        writeCommand[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }
    SPI_Write(AD7193_SLAVE_ID * modifyCS, writeCommand, bytesNumber + 1);// write data to SPI
}//setregistervalue

/*! Reads the value of a register. */
unsigned long AD7193_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber,
                                      unsigned char modifyCS)

{//getregistervalue
    unsigned char registerWord[5] = {0, 0, 0, 0, 0}; 
    unsigned long buffer          = 0x0;
    unsigned char i               = 0;
    
    registerWord[0] = AD7193_COMM_READ |
                      AD7193_COMM_ADDR(registerAddress);
    SPI_Read(AD7193_SLAVE_ID * modifyCS, registerWord, bytesNumber + 1);
    for(i = 1; i < bytesNumber + 1; i++) 
    {
        buffer = (buffer << 8) + registerWord[i];
    }
    
    return buffer;

}//getregistervalue

/*! Selects the AD7193's operating mode. */
void AD7193_SetMode(unsigned char mode)
{//setmode
    unsigned long oldRegValue = 0;
    unsigned long newRegValue = 0;

    oldRegValue = AD7193_GetRegisterValue(AD7193_REG_MODE, 3, 1);
    oldRegValue &= ~AD7193_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7193_MODE_SEL(mode);
    AD7193_SetRegisterValue(AD7193_REG_MODE,newRegValue,3,1);                     
}//setmode

/*! Resets the device. */
void AD7193_Reset(void)

{//reset
    unsigned char registerWord[6] = {0, 0, 0, 0, 0, 0};
    registerWord[0] = 0xFF;
    registerWord[1] = 0xFF;
    registerWord[2] = 0xFF;
    registerWord[3] = 0xFF;
    registerWord[4] = 0xFF;
    registerWord[5] = 0xFF;
    SPI_Write(AD7193_SLAVE_ID, registerWord, 6);

}//reset

/*! Set device to idle or power-down. */
void AD7193_SetPower(unsigned char pwrMode)
{//setpower
     unsigned long oldPwrMode = 0x0;
     unsigned long newPwrMode = 0x0; 
 
     oldPwrMode = AD7193_GetRegisterValue(AD7193_REG_MODE, 3, 1);
     oldPwrMode &= ~(AD7193_MODE_SEL(0x7));
     
     newPwrMode = oldPwrMode | 
                  AD7193_MODE_SEL((pwrMode * (AD7193_MODE_IDLE)) |
                                  (!pwrMode * (AD7193_MODE_PWRDN)));

     AD7193_SetRegisterValue(AD7193_REG_MODE, newPwrMode, 3, 1);  

}//setpower

/*! Waits for RDY pin to go low. */
void AD7193_WaitRdyGoLow(void)

{//WaitRdyGoLow
    while(digitalRead(AD7193_RDY_STATE));
}//WaitRdyGoLow

/*! Selects the channel to be enabled. */
void AD7193_ChannelSelect(unsigned short channel)

{//ChannelSelect

    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   

    oldRegValue = AD7193_GetRegisterValue(AD7193_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7193_CONF_CHAN(0x3FF));
    newRegValue = oldRegValue | AD7193_CONF_CHAN(1 << channel);   
    AD7193_SetRegisterValue(AD7193_REG_CONF, newRegValue, 3, 1);

}//ChannelSelect

/*! Performs the given calibration to the specified channel. */
void AD7193_Calibrate(unsigned char mode, unsigned char channel)

{//Calibrate

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
}//Calibrate


/*! Selects the polarity of the conversion and the ADC input range. */
void AD7193_RangeSetup(unsigned char polarity, unsigned char range)

{//RangeSetup

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

}//RangeSetup

unsigned long AD7193_SingleConversion(void)

{//SingleConversion
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

    return regData;// return data of single convertion on selected channels

}//SingleConversion

/*! Returns the average of several conversion results. */
unsigned long AD7193_ContinuousReadAvg(unsigned char sampleNumber)

{//ContinuousReadAvg
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
    return samplesAverage;// return average of "samplenumber" convertion on selected channel
}//ContinuousReadAvg


/*! Read data from temperature sensor and converts it to Celsius degrees. */
unsigned long AD7193_TemperatureRead(void)

{//TemperatureRead
    unsigned long dataReg     = 0;
    unsigned long temperature = 0;     
    AD7193_RangeSetup(0, AD7193_CONF_GAIN_1); // Bipolar operation, 0 Gain.
    AD7193_ChannelSelect(AD7193_CH_TEMP);// select the temperature sensor
    dataReg = AD7193_SingleConversion();// return data of single convertion in temperature sensor
    dataReg -= 0x800000;
    temperature = dataReg / 2815;    // Kelvin Temperature
    temperature -= 273;              // Celsius Temperature
    return temperature;// return value of temperature 
}//TemperatureRead


/*! Converts 24-bit raw data to volts. */
float AD7193_ConvertToVolts(unsigned long rawData, float vRef)

{//ConvertToVolts
    float voltage = 0;
    if(currentPolarity == 0 )   // Bipolar mode
    {
        voltage = (((float)rawData / (1ul << 23)) - 1) * vRef / currentGain;
    }
    else                        // Unipolar mode
    {
        voltage = ((float)rawData * vRef) / (1ul << 24) / currentGain;
    }
    return voltage;// return rawData in volt according to vRef
}//ConvertToVolts

unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg)
{//SPI_Init
    unsigned char status = 1;
    unsigned long pbFrequency = 80000000;
    unsigned short brgValue = 0;

    digitalWrite(SS,HIGH);//SPI_CS_HIGH;
    pinMode(SS,OUTPUT);//SPI_CS_PIN_OUT;
    /*!< Fsck = Fpb / (2 * (SPIxBRG + 1)) */
    brgValue = pbFrequency / (2 * clockFreq) - 1;
    SPI2CON = 0; /*!< Clear the content of SPI2CON register */
    SPI2BRG = brgValue;
    SPI2CONbits.CKE = clockEdg; /*!< SPI Clock Edge Select bit */
    SPI2CONbits.CKP = clockPol; /*!< Clock Polarity Select bit */
    SPI2CONbits.MSTEN = 1; /*!< Enable master mode */
    SPI2CONbits.ON = 1; /*!< Enable SPI peripheral */

    return status;
}//SPI_Init

unsigned char SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{//SPI_Read
    unsigned char byte = 0;
    unsigned char writeBuffer[4] = {0, 0, 0, 0};

    for(byte = 0; byte < bytesNumber; byte++)
    {
        writeBuffer[byte] = data[byte];
    }
    if(slaveDeviceId == 1)
    {
       digitalWrite(SS,LOW);
    }
    for(byte = 0; byte < bytesNumber; byte++)
    {
        SPI2BUF = writeBuffer[byte];
        while(SPI2STATbits.SPIRBF == 0);
        data[byte] = SPI2BUF;
    }
    while(SPI2STATbits.SPIBUSY == 1);
    if(slaveDeviceId == 1)
    {
        digitalWrite(SS,HIGH);
    }
    return bytesNumber;
}//SPI_Read



unsigned char SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber)
{//SPI_Write
    unsigned char byte = 0;
    unsigned char tempByte = 0;

    if(slaveDeviceId == 1)
    {
        digitalWrite(SS,LOW);
    }
    for(byte = 0; byte < bytesNumber; byte++)
    {
        SPI2BUF = data[byte];
        while(SPI2STATbits.SPIRBF == 0);
        tempByte = SPI2BUF;
    }
    while(SPI2STATbits.SPIBUSY == 1);
    if(slaveDeviceId == 1)
    {
        digitalWrite(SS,HIGH);
    }
    return bytesNumber;
}//SPI_Write


void setup()
{//setup
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
    /*Set the pseudo bit in configuration register */
    regValue = AD7193_GetRegisterValue(AD7193_REG_CONF, 3, 1);  
    regValue |= AD7193_CONF_PSEUDO;
    AD7193_SetRegisterValue(AD7193_REG_CONF, regValue, 3, 1);
    
}//setup
void loop()
{ 
  unsigned long data = 0;
  data = AD7193_ContinuousReadAvg(1000);//retur an average of 1000 convertion on selected channel
  float volt=AD7193_ConvertToVolts(data,-3.3);// convert binari data to volt
  delay(1000);// wait a while
  Serial.print("volt=");
  Serial.println(volt,DEC);// print the voltage of selected channel

}
