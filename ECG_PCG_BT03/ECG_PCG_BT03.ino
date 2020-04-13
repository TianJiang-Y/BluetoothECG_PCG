//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

#include "driver/i2s.h"
#include "driver/uart.h"
#include "esp_system.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


// For I2S/WM8960 pin mapping
const int i2s_lrclk = 25;
const int i2s_adc = 19;
const int i2s_bclk = 21;

const int sclk = 14;
const int sdin = 13;

// For OLED pin mapping
const int oledcs = 22;
const int oleddc = 23;
const int oledrest = 18;
const int oledsda = 5;
const int oledscl = 4;

// For Battery Charger Guard
const int nstdby = 12;
const int nchrg = 15;

// For ECG
const int ecg_powen = 27;
const int ecg_rxd = 16;
const int ecg_txd = 17;

/* WM8960 Control Interface- I2C communication */
#define WM_SDA_IN()            pinMode(sdin, INPUT);
#define WM_SDA_OUT()           pinMode(sdin, OUTPUT);
#define WM_SCL_OUT()           pinMode(sclk, OUTPUT);

#define WM_SDA_HIGH()          digitalWrite(sdin, HIGH);   // Sets SDA line
#define WM_SDA_LOW()           digitalWrite(sdin, LOW);    // Clears SDA line
#define WM_SDA()               digitalRead(sdin);

#define WM_SCL_HIGH()          digitalWrite(sclk, HIGH);  // Sets SCL line
#define WM_SCL_LOW()           digitalWrite(sclk, LOW);  // Clears SCL line
//***************************************************************************

/* R25 - Power 1 */
#define WM8960_VMID_MASK     0x180
#define WM8960_VREF          0x40

/* R26 - Power 2 */
#define WM8960_PWR2_LOUT1    0x40
#define WM8960_PWR2_ROUT1    0x20
#define WM8960_PWR2_OUT3     0x02

/* R28 - Anti-pop 1 */
#define WM8960_POBCTRL       0x80
#define WM8960_BUFDCOPEN     0x10
#define WM8960_BUFIOEN       0x08
#define WM8960_SOFT_ST       0x04
#define WM8960_HPSTBY        0x01

/* R29 - Anti-pop 2 */
#define WM8960_DISOP         0x40
#define WM8960_DRES_MASK     0x30

#define SAMPLE_RATE     (8000)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (100)
#define PI              (3.14159265)

#define I2S_BCK_IO      (GPIO_NUM_21) // BCLK
#define I2S_WS_IO       (GPIO_NUM_25) // LRCLK
#define I2S_DO_IO       (-1)          // Not used
#define I2S_DI_IO       (GPIO_NUM_19) // ADC

#define UART2_TXD       (GPIO_NUM_17)
#define UART2_RXD       (GPIO_NUM_16)

#define UART2_RTS       (-1)
#define UART2_CTS       (-1)

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

#define WM8960_ADDRESS        0x1a // = 0x34 << 1

#define WM8960_I2C_Address    0x34

uint32_t AudioTotalSize;  /* This variable holds the total size of the audio file */
uint32_t AudioRemSize;    /* This variable holds the remaining data in audio file */
uint16_t *CurrentPos;     /* This variable holds the current position of audio pointer */

int16_t PCG_CodecData;
uint8_t BT_DataPcket[20];

uint64_t chipid;
uint16_t tempID;
uint16_t chipidLSB;
char DeviceName[20];

//WM8960 resgister value
static uint16_t WM8960_REG_VAL[56] =
{  
  0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000A,
  0x01C0, 0x0000, 0x00FF, 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x007B, 0x0100, 0x0032, 0x0000, 0x00C3, 0x00C3, 0x01C0,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000,
  0x0000, 0x0037, 0x004D, 0x0080, 0x0008, 0x0031, 0x0026, 0x00ED
};

//******************************************************************
// GPIO I2C Communication part.
//******************************************************************

static void WM_i2c_Delay(void) // 526KHz max
{
    delayMicroseconds(2);
}

static unsigned char WM_SDA_ReadBit(void)
{
    unsigned char bitVal;
    unsigned char sda_bit;

    sda_bit = WM_SDA();
    
    if(sda_bit) // Read bit, save it in bit
        bitVal=1;
    else
        bitVal=0;
    
    return bitVal;
}

static void WM_i2c_Start(void)
{
    WM_SDA_OUT();
    WM_SDA_HIGH();
    WM_i2c_Delay();
    WM_SCL_HIGH();
    WM_i2c_Delay();
    WM_SDA_LOW();
    WM_i2c_Delay();
    WM_SCL_LOW();
    WM_i2c_Delay();
}

static void WM_i2c_Stop(void)
{
    WM_SDA_OUT();
    WM_SCL_LOW();
    WM_i2c_Delay();
    WM_SDA_LOW();
    WM_i2c_Delay();
    WM_SCL_HIGH();
    WM_i2c_Delay();
    WM_SDA_HIGH();
}

static void WM_i2c_Ack(void)
{
    WM_SCL_LOW();
    WM_SDA_OUT();
    WM_i2c_Delay();
    WM_SDA_LOW();
    WM_i2c_Delay();
    WM_SCL_HIGH();
    WM_i2c_Delay();
    WM_SCL_LOW();
}

static void WM_i2c_NAck(void)
{
    WM_SCL_LOW();
    WM_SDA_OUT();
    WM_i2c_Delay();
    WM_SDA_HIGH();
    WM_i2c_Delay();
    WM_SCL_HIGH();
    WM_i2c_Delay();
    WM_SCL_LOW();
}

static void WM_i2c_SendByte(unsigned char _ucByte)
{
  unsigned char i;
  WM_SDA_OUT();
  WM_SCL_LOW();
  WM_i2c_Delay();

  for (i = 0; i < 8; i++)
  {   
    if (((_ucByte & 0x80) >> 7) & 0x01)
    {
      WM_SDA_HIGH();
    }
    else
    {
      WM_SDA_LOW();
    }
        
    _ucByte <<= 1;       
//    WM_i2c_Delay();
    WM_SCL_HIGH();
    WM_i2c_Delay(); 
    WM_SCL_LOW();
    WM_i2c_Delay();
  }
}

static unsigned char WM_i2c_ReadByte(unsigned char ack)
{
  unsigned char i;
  unsigned char value = 0;
  unsigned char nbit;
  
  WM_SDA_IN();
  WM_i2c_Delay();

  for (i = 0; i < 8; i++)
  {
    WM_SCL_LOW();
    WM_i2c_Delay();
    WM_SCL_HIGH();
        
    value <<= 1;
        
    nbit = WM_SDA_ReadBit();
        
    if (nbit)
    {
      value++;
    }
  }

  if(ack == 0)
    WM_i2c_NAck();
  else
    WM_i2c_Ack();
    
  return value;
}

static unsigned char WM_i2c_WaitAck(void)
{
    unsigned char re;
    unsigned char ucErrTime = 0;

    WM_SDA_IN(); // SDA-input
    WM_SCL_HIGH();
    WM_i2c_Delay();
    
    re = WM_SDA_ReadBit();

    while (re)
    {
        ucErrTime++;

        if(ucErrTime > 150)
        {
            WM_i2c_Stop();
            return 1;
        } 

        WM_i2c_Delay();
        re = WM_SDA_ReadBit();
    }

    WM_SCL_LOW();

    return 0;    
}

static void WM_I2C_GPIO_Init(void)
{   
    WM_SCL_OUT();
    WM_SCL_HIGH();
    WM_SDA_OUT();
    WM_SDA_HIGH();
}
/**
  * @brief  Write register of WM8960.
  * @param  reg: The number of resigter which to be read.
  * @param  dat: The data which will be writeen to the register.
  * @retval The value of regsiter.
  */
static void WM8960_Write_Reg(unsigned char reg_add, uint16_t reg_data)
{
    unsigned char res, I2C_Data[2];

    I2C_Data[0] = (reg_add << 1) | ((unsigned char)((reg_data >> 8) & 0x0001));
    I2C_Data[1] = (unsigned char)(reg_data & 0x00FF);
    
    WM_i2c_Start();
    WM_i2c_SendByte(WM8960_I2C_Address);
    WM_i2c_WaitAck();
    WM_i2c_SendByte(I2C_Data[0]);
    WM_i2c_WaitAck();
    WM_i2c_SendByte(I2C_Data[1]);
    WM_i2c_WaitAck();
    WM_i2c_Stop();
}

//*****************************************************************
void WM8960_Init(void)  
{
    uint8_t res;
    
    //Reset Device
    WM8960_Write_Reg(0x0F, 0x0000);
    delay(10);    
/**********************************************************************/   
    // Set Power Source
 /**************************************************************************/
    //*** < Power Management1 -R25 Register Input Power > ***
    // VMIDSEL[1:0] = 11, 2 x 5kOHm divider enabled (for fast start-up)
    // VREF = 1, VREF (necessary for all other functions)= Power up
    // AINL = 1, Analogue in PGA Left = Power Up
    // AINR = 0, Analogue in PGA Right = Power Down
    // ADCL = 1, ADC Left = Power Up
    // ADCR = 0; ADC Right = Power Down
    // MICBIAS = 1, Power Up
    // DIGENB = 0, Master clock enabled
//    WM8960_Write_Reg(0x19, 0x01EA); // 2*0KOhm(for fast start-up)
    WM8960_Write_Reg(0x19, 0x016A); // 2*50KOhm(for playback/record)
    
    //*** < Power Management2 R26 Register - Output Power > ***
    // DACL = 1, DAC Left = Power Up
    // DACR = 0, Power Down
    // LOUT1 = 0, Power Down
    // ROUT1 = 0, Power Down
    // SPKL = 0, Power Down
    // SPKR = 0, Power Down
    // bit2 = 0, reserved
    // OUT3 = 1, OUT3 Output Buffer = Power Up
    // PLL_EN = 1, PLL Power Up
//    WM8960_Write_Reg(0x1A, 0x0103); // for debug using OUT3
    WM8960_Write_Reg(0x1A, 0x0101); // DACL and PLL_EN only Power up

    //*** < Power Management3 R47 - Input & Output Signal Path Setting > ***
    // LMIC = 1 (if AINL = 1), Left Channel Input PGA Enable
    // LOMIX = 1, Left Output Mixer Enable Control
    WM8960_Write_Reg(0x2F, 0x0028);
    
 /************************************************************************/   
    // Configure clock
 /************************************************************************/
    // MCLK->div2->f1->PLL->f2 = 65.536MHz -> f/4 -> SYSCLKDIV Div2
    // ***PLL1 R52 Register***
    // SDM = 1, Fractional mode
    // PLLPRESCALE = 1, Divide by 2
    // PLLN[3:0] = 5h
    WM8960_Write_Reg(0x34, 0x0035); // SYSCLK = 8.192MHz
    // PLLK = 7619F0h
    WM8960_Write_Reg(0x35, 0x0076); // R53[5:0] = 76h
    WM8960_Write_Reg(0x36, 0x0019); // R54[8:0] = 19h
    WM8960_Write_Reg(0x37, 0x00F0); // R55[8:0] = F0h
    
    // ADCDIV = 100, SYSCLK / (4*256)
    // DACDIV = 100, SYSCLK / (4*256)
    // SYSCLKDIV = 10, Divide SYSCLK by 2
    // CLKSEL = 1, SYSCLK derived from PLL output
    WM8960_Write_Reg(0x04, 0x125); // ADC/DAC Sample rate = 8KHz

    // Configure the Bit Clock of I2S interface
    // Audio Interface Register R7
    // MS = 1, Enable master mode
    // WL[1:0] = 00, 16bit Audio Data Word Length
    // FORMAT[1:0] = 10, I2S Format
    WM8960_Write_Reg(0x07, 0x0042);

    // Clocking2 Register
    // DCLKDIV[2:0] = default 111, Class D switching clock divider
    // BCLKDIV[3:0] = 1111, BCLK Frequency = SYSCLK / 32
    WM8960_Write_Reg(0x08, 0x01CF); // BCLK Frequency(Master Mode)= 256KHz
                                    // LRCLK = BCLK / 32 = 8KHz
 /**************************************************************************/
    // Left Input PGA Configurations
    // Single-ended MIC configuration on left channel
 /**************************************************************************/
    //*** < Left Input Volume R0 Register > ***
    // IPVU = 0, Input PGA Volume Update
    // LINMUTE = 0, Left Input Disable Mute
    // LINVOL = 111111, +30dB, default (0b010111)-------------amp
    WM8960_Write_Reg(0x00, 0x013F);
//    WM8960_Write_Reg(0x00, 1<<8); // default
    
    //*** < ADCL Signal Path R32 Register > ***
    // LMN1 = 1, LINPUT1 connected to PGA
    // LMP3 = 0
    // LMP2 = 0
    // LMICBOOST[1:0] = 10, +20dB----------------------------------amp
    //                  00, 0dB
    //                  01, +13dB
    //                  11, +29dB
    // LMIC2B = 1, Connect Left Input PGA to Input Boost Mixer
    // bit2-bit0 = 000
    WM8960_Write_Reg(0x20, 0x0138);
    
    //*** < Left Input Boost Mixer R43 Register > ***
    // LIN2BOOST = 000, Mute
    // LIN3BOOST = 000, Mute
    WM8960_Write_Reg(0x2B, 0x0000);
    
/*********************************************************************/
//            ALC COntrol
/********************************************************************/

    //*** < ALC(1) R17 Register > ***
    // ALCSEL[1:0] = 10, Left channel only
    // MAXGAIN[2:0] = 000 -12dB
    //                111 +30dB default
    // ALCL[3:0] =   1010 -6dB Fs
    //               1011 default -12dB Fs
//    WM8960_Write_Reg(0x11, 0x0000); // ALC Disable
    WM8960_Write_Reg(0x11, 0x017B); // 0dB Fs
    
    //*** < ALC(2) R18 Register > ***
    // bit8 = 1, default, reserved
    // bit7 = 0, reserved
    // MINGAIN[2:0] = 000 -17.25dB default
    //                111 +24.75dB
    // HLD[3:0] =   0000 0mS default
    //              1111 43.961S
    WM8960_Write_Reg(0x12, 0x0100); // Hold Time = 5.33mS
    
    //*** < ALC(3) R19 Register > ***
    // ALCMODE = 0, ALC Mode
    //           1, Limiter Mode
    // DCY[3:0] = 0011, default, 192mS
    //            0000, 24mS
    //            1010 or Higher 24.58S
    // ATK[3:0] = 0010, default, 24mS
    //            0000, 6mS
    //            1010 or Higher 6.14S
    WM8960_Write_Reg(0x13, 0x0000);

    //*** < Additional Control R27 Register > ***
    // bit8, bit7 = 00, reserved
    // VROI = 1, 20KOhm VMID to output
    //        0, 500Ohm VMID to output
    // bit5, bit4 = 00, reserved
    // OUT3CAP = 0, OUT3 enabled and disabled toghter with HP_L and HP_R
    // ADC_ALC_SR = 101, 8K ALC Sample Rate
    // (ADC_ALC_SR should be congigured to match the chosen ADC sample rate)
    WM8960_Write_Reg(0x1B, 0x0005);
 
    // Noise Gate R20 Register
    // bit8 = 0, reserved
    // NGTH[4:0] = 00011, Noise gate threshold
    //             00000, default
    // bit2, bit1 = 00, default, reserved
    //              01 ????????????????????????????????????????????
    // NGAT = 1, Noise gate function enable
    
//    WM8960_Write_Reg(0x14, 0x0079); // Threshold = -76.5dBfs default
    WM8960_Write_Reg(0x14, 0x00F9); // Threshold = -30dBfs default
 /*********************************************************************/
    // Configure ADC/DAC
 /*********************************************************************/
    //*** < ADC/DAC Control 1 R5 Register > ***
    // DACDIV2 = 0 (0dB), DAC 6dB Attenuate Enable Disabled
    // ADCPOL[1:0] = 00, Polarity no inverted
    // DACMU = 0, DAC Digital Soft No Mute(signal active)>>>>>>>>>>>>>>>>>>
    // DEEMPH[1:0] = 00, No de-emphasis
    // ADCHPD = 1, Disable high pass filter in left and right channels>>>>>
    WM8960_Write_Reg(0x05, 0x0001);
    
    //*** < Left ADC Volume R21 Register > ***
    // ADCVU = 1, Writing 1 to this bit will cause left and right ADC volumes to be updated
    // LADCVOL[7:0] = 1111 1111, +30dB
    //                1100 0011, default, 0dB
    //                0000 0000, Digital Mute
    WM8960_Write_Reg(0x15, 0x01C3); // +30dB--------------------Main AMP
    
    //*** < ADC/DAC Control 2 R6 Register > ***
    // DACPOL[1:0] = 00, DAC Polarity not inverted
    // DACSMM = 0, DAC Soft Mute Mode Immediately
    // DACMR = 1, Soft mute ramp rate = Slow ramp
    // DACSLOPE = 0, DAC Filter normal mode
    WM8960_Write_Reg(0x06, 0x0004);

    //*** < Left DAC Volume R10 Register > ***
    // DACVU = 1, DAC Volume Update
    // LDACVOL[7:0] = 1111 1111 default, 0dB
    //                0000 0001 -127dB
    WM8960_Write_Reg(0x0A, 0x01FF);
 /*********************************************************************/
    // Left Out Configuration
    // For Debug, output the Input analog signal to OUT3
 /*********************************************************************/
    //*** < Left Out Mix R34 Register > ***
    // LD2LO = 1, Left DAC to Left Output Mixer Enable Path
    // LI2LO = 0, LINPUT3 to Left Output Mixer Disable(Mute)
    // LI2LOVOL[2:0] = 000, Open when LI2LOVOL = 000
    WM8960_Write_Reg(0x22, 0x0100);
    
    //*** < Left Bypass R45 Register > ***
    // LB2LO = 1, 
    // LB2LOVOL = 101 default, 000 = 0dB
    //WM8960_Write_Reg(0x2D, 0x0080);
    
    //*** < Mono Out Mix1 R38 Register > ***
    // L2MO = 1, left Channel mix enabled
    WM8960_Write_Reg(0x26, 0x0080);
    
    //*** < OUT3 Volume R42 Register > ***
    // MOUTVOL = 0, 0dB : 1 = -6dB
    WM8960_Write_Reg(0x2A, 0x0040); // 0dB
    //WM8960_Write_Reg(0x2A, 1<<6);   // -6dB
    
    //*** < Audio Innterface R9 Register > ***
    // LOOPBACK = 1, LoopBack Enable
//    WM8960_Write_Reg(0x09, 0x0001); // Enable
    WM8960_Write_Reg(0x09, 0x0000); // Disable----------------------------
    delay(1000);
}

//*****************************************************************
//                  i2s configuration
//*****************************************************************
int i2s_num = 0; // i2s port number
int16_t I2S_buffer[32] = {0};
uint8_t I2S_PCGPacket[8] = {0};

i2s_config_t i2s_config = {
     .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
     .sample_rate = 8000,
     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
     .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S),
     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority(Level 1)
     .dma_buf_count = 8,  // number of buffers, 128 max
     .dma_buf_len = 64, // size of each buffer
     .use_apll = false
};

//updated I2S pins for tAudio
i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_IO, //this is BCK pin
    .ws_io_num = I2S_WS_IO, // this is LRCK pin
    .data_out_num = -1, // No use, this is DATA output pin
    .data_in_num = I2S_DI_IO   //
};

uint32_t missedByteCounter = 0; //debug variable

void ESP32_i2sSetup(void)
{
    // initialize i2s with configurations above
    i2s_driver_install((i2s_port_t)I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin((i2s_port_t)I2S_NUM, &pin_config);

    // set sample rates of i2s to samplerate of heat rate data
    i2s_set_sample_rates((i2s_port_t)i2s_num, 8000);
}

void PCG_I2S_Reader(void)
{
    size_t bytes_read;
    uint32_t read_counter = 0;
    
    i2s_read(I2S_NUM_0, &I2S_buffer, 64, &bytes_read, 100);

    //if(bytes_read == 32)
    //{
        return;
        //PCG_CodecData = I2S_buffer[7];
        //read_counter++;
    //}
}

//**********************************************************************
//            UART2 Configuration
//**********************************************************************

const int uart_num = UART_NUM_2;

// Setup UART buffered IO with event queue
const int uart_buffer_size = (1024 * 2);
//QueueHandle_t uart_queue;

// Read data from UART.
//uint8_t* UART_ECGPacket = (uint8_t*)malloc(uart_buffer_size);
//uint8_t* UART_ECGPacket = (uint8_t*)malloc(10);

uint8_t UART_ECGPacket[20] = {0};
uint16_t length;

uart_config_t uart_config = {
    .baud_rate = 57600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};

void ESP32_UART2setup(void)
{
    // Configure UART parameters
    uart_param_config(UART_NUM_2, &uart_config);

    //Serial2.begin(57600, SERIAL_8N1, UART2_RXD, UART2_TXD);
    //Serial2.available();
    
    // Set UART pins
    // TX: IO17 (UART2 default)
    // RX: IO16 (UART2 default)
    // RTS: IO18, CTS: IO19
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);

    // Install UART driver (we don't need an event queue here)
    // In this program we don't even use a buffer for sending data
    uart_driver_install(UART_NUM_2, uart_buffer_size, 0, 0, NULL, 0);
}

uint8_t ECG_UART_Reader(void)
{
    uint8_t k = 0, i = 0;
    uint8_t ByteLength = 6;
    uint8_t ByteCount = 0;
    uint8_t CheckSum = 0;
    
    // Read data from UART.
    length = 0;
    UART_ECGPacket[20] = {0};
    
//    length = uart_read_bytes(UART_NUM_2, UART_ECGPacket, uart_buffer_size, 100);
    length = uart_read_bytes(UART_NUM_2, UART_ECGPacket, 20, 120);

    for(k = 0; k < 20; k++)
    {
        if(UART_ECGPacket[k] == 0xAA)
        {
            if((k <= 20 - ByteLength) & (UART_ECGPacket[k + 1] == 0xAA)) 
            {              
                ByteCount = k + 2;
                
                if((UART_ECGPacket[ByteCount] == 0x04) & 
                   (UART_ECGPacket[ByteCount + 1] == 0x80))
                  {
                      for(i = 1; i < 5; i++)
                          CheckSum += UART_ECGPacket[ByteCount + i];
                      CheckSum &= 0xFF;
                      CheckSum = ~CheckSum & 0xFF;
  
                      if(CheckSum == UART_ECGPacket[k + 7])
                          return ByteCount;
                      else
                          return 0;
                  }
                else
                  return 0;
            }
            else
              return 0;
          }
      }
}

//************************************************************************

void ECG_PowerEnable(void)
{
    digitalWrite(ecg_powen, HIGH);
}

void ECG_PowerDisable(void)
{
   digitalWrite(ecg_powen, LOW);
}

void setup() {
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    delay(1000);delay(1000);
    
    Serial.begin(115200);
//    Serial2.begin(57600, SERIAL_8N1, UART2_RXD, UART2_TXD);
    
    //The chip ID is essentially its MAC address(length: 6 bytes).
    chipid = ESP.getEfuseMac();

    Serial.println("Serial Connection OK!");
    Serial.printf("ESP32 Chip ID = %X\n",(uint16_t)(chipid>>32));//print High 2 bytes
    Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

    tempID = (uint16_t)(chipid>>32);
    chipidLSB = ((uint8_t)(tempID) << 8 | (uint8_t)(tempID >> 8)) + 0x0002;
    Serial.printf("MAC-ID = %X\n", chipidLSB);
//    Serial.printf("ESP32 Chip ID2 = %x\n",(uint16_t)(chipid>>48));//print High 2 bytes
    
    pinMode(ecg_powen, OUTPUT);
    ECG_PowerEnable();
    delay(100);
  
    WM_I2C_GPIO_Init();
    WM8960_Init();
    delay(3000);
  
    ESP32_i2sSetup();
    delay(10);
    
    ESP32_UART2setup();
    delay(10);
    
    ///////////ECG_PowerDisable();
    sprintf(DeviceName, "BTSmartE&PCG-%04X", chipidLSB);

    SerialBT.begin(DeviceName); //Bluetooth device name
      
    Serial.println("Initialize OK!");
    Serial.println(DeviceName);
    Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
    uint16_t i = 0;
    uint8_t cnt = 0;
    BT_DataPcket[20] = {0};
    
    // put your main code here, to run repeatedly:
    
      do
      {
        cnt = 0;
        cnt = ECG_UART_Reader();
      }while(!(cnt > 1));
      
      PCG_I2S_Reader();
      
      BT_DataPcket[0] = 0xAA; // 0xAA
      BT_DataPcket[1] = 0xAA; // 0xAA
      BT_DataPcket[2] = UART_ECGPacket[cnt + 3]; // data MSB
      BT_DataPcket[3] = UART_ECGPacket[cnt + 4]; // data LSB
      BT_DataPcket[4] = 0xDD;      
      BT_DataPcket[5] = 0xDD;        
      
      BT_DataPcket[6] = 0xBB;
      BT_DataPcket[7] = 0xBB;
      BT_DataPcket[8] = (I2S_buffer[7] >> 8);  // MSB
      BT_DataPcket[9] = (uint8_t)I2S_buffer[7]; // LSB
      BT_DataPcket[10] = (I2S_buffer[15] >> 8);
      BT_DataPcket[11] = (uint8_t)I2S_buffer[15];
      BT_DataPcket[12] = (I2S_buffer[31] >> 8);
      BT_DataPcket[13] = (uint8_t)I2S_buffer[31];        
      BT_DataPcket[14] = 0xEE;
      BT_DataPcket[15] = 0xEE;

    //if (SerialBT.available())
      SerialBT.write(BT_DataPcket, 16);
}
