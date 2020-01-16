/**
  ******************************************************************************
  * @file    Keyfob/CC1101.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   CC1101 transceiver low level functions.
  ******************************************************************************
  * @attention
  * Copyright   : Ebyte electronic co.,LTD
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#include "CC1101.h"
#include "bsp_rf.h"

typedef struct {
    uint8_t registerAddress;
    uint8_t registerValue;
} RfSetting;

// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// GDO2 signal selection = (41) CHIP_RDY

// Address Config = Address check and 0 (0x00) broadcast 
// Base Frequency = 432.999817 
// CRC Autoflush = false 
// CRC Enable = true 
// Carrier Frequency = 432.999817 
// Channel Number = 0 
// Channel Spacing = 199.951172 
// Data Format = Normal mode 
// Data Rate = 9.59587 
// Deviation = 19.042969 
// Device Address = 1 
// Manchester Enable = false 
// Modulated = true 
// Modulation Format = GFSK 
// PA Ramping = false 
// Packet Length = 255 
// Packet Length Mode = Variable packet length mode. Packet length configured by the first byte after sync word 
// Preamble Count = 4 
// RX Filter BW = 203.125000 
// Sync Word Qualifier Mode = 30/32 sync word bits detected 
// TX Power = 0 
// Whitening = false 
// PA table 
#define PA_TABLE {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00} //0xc0

static const RfSetting rfSetting[]= 
{
    {CC1101_IOCFG0,      0x06},
    {CC1101_FIFOTHR,     0x47},
    {CC1101_PKTCTRL1,    0x06},
    {CC1101_PKTCTRL0,    0x05},
    {CC1101_ADDR,        0x01},
    {CC1101_CHANNR,      0x00},
    {CC1101_FREQ2,       0x10},
    {CC1101_FREQ1,       0xA7},
    {CC1101_FREQ0,       0x62},
    {CC1101_MDMCFG4,     0x88},// 0x28
    {CC1101_MDMCFG3,     0x83},
    {CC1101_MDMCFG2,     0x13},
    {CC1101_DEVIATN,     0x34},
    {CC1101_FSCAL3,      0xE9},
    {CC1101_FSCAL2,      0x2A},
    {CC1101_FSCAL1,      0x00},
    {CC1101_FSCAL0,      0x1F},
    {CC1101_TEST2,       0x81},
    {CC1101_TEST1,       0x35},
    {CC1101_TEST0,       0x09},
    {CC1101_FSCTRL1,     0x06},
                   //{CC1101_FOCCFG,      0x16},
                   //{CC1101_AGCCTRL2, 0xC7}, //  AGC control. reset value=  03
    //{CC1101_AGCCTRL1, 0x4b}, //  AGC control. reset value= 40   4b - decrese CS abs  threshold
                   //{CC1101_AGCCTRL0, 0xB0}, //  AGC control. 91
};


// Chipcon
// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 200 kHz //541.666667 kHz
// Deviation = 19 kHz //127 kHz
// Datarate = 9.6 kBaud //249.938965 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 432.999817 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = 10b address check and broadcast addr = 0 //(0) No address check
// FIFO autoflush = 0
// Device address = 1 //0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (41) CHIP_RDY
/*
static const RfSetting rfSetting[]= 
{
    {CC1101_FSCTRL1 , 0x0C}, //  Frequency synthesizer control.
    {CC1101_FSCTRL0 , 0x00}, //  Frequency synthesizer control.
    {CC1101_FREQ2   , 0x10}, //  Frequency control word, high byte.
    {CC1101_FREQ1   , 0xA7}, //  Frequency control word, middle byte.
    {CC1101_FREQ0   , 0x62}, //  Frequency control word, low byte.
    {CC1101_MDMCFG4 , 0x88}, // was 28 was 2d Modem configuration.
    {CC1101_MDMCFG3 , 0x83}, // was 3b Modem configuration.
    {CC1101_MDMCFG2 , 0x13}, //  Modem configuration.
    {CC1101_MDMCFG1 , 0x22}, //  Modem configuration.
    {CC1101_MDMCFG0 , 0xF8}, //  Modem configuration.
    {CC1101_CHANNR  , 0x00}, //  Channel number.
    {CC1101_DEVIATN , 0x34}, //  was 62 Modem deviation setting (when FSK modulation is enabled).
    {CC1101_FREND1  , 0xB6}, //  Front end RX configuration.
    {CC1101_FREND0  , 0x10}, //  Front end TX configuration.
    {CC1101_MCSM0   , 0x18}, //  Main Radio Control State Machine configuration.
    {CC1101_FOCCFG  , 0x1D}, //  Frequency Offset Compensation Configuration. Reset value = 36
    {CC1101_BSCFG   , 0x1C}, //  Bit synchronization Configuration.
    {CC1101_AGCCTRL2, 0xC7}, //  AGC control. reset value=  03
    {CC1101_AGCCTRL1, 0x00}, //  AGC control. reset value= 40   4b - decrese CS abs  threshold
    {CC1101_AGCCTRL0, 0xB0}, //  AGC control. 91
    {CC1101_FSCAL3  , 0xE9}, //  was ea Frequency synthesizer calibration.
    {CC1101_FSCAL2  , 0x2A}, //  Frequency synthesizer calibration.
    {CC1101_FSCAL1  , 0x00}, //  Frequency synthesizer calibration.
    {CC1101_FSCAL0  , 0x1F}, //  Frequency synthesizer calibration.
    {CC1101_FSTEST  , 0x59}, //  Frequency synthesizer calibration.
    {CC1101_TEST2   , 0x81}, //  was 88 Various test settings.
    {CC1101_TEST1   , 0x35}, //  was 31 Various test settings.
    {CC1101_TEST0   , 0x09}, //  Various test settings.
    {CC1101_FIFOTHR , 0x47}, //  was 07 RXFIFO and TXFIFO thresholds.
    {CC1101_IOCFG2  , 0x29}, //  GDO2 output pin configuration.
    {CC1101_IOCFG0  , 0x06}, //  GDO0 output pin configuration. 
    {CC1101_PKTCTRL1, 0x06}, // was 04 Packet automation control.
    {CC1101_PKTCTRL0, 0x05}, //  Packet automation control.
    {CC1101_ADDR    , 0x01}, //  was 00 Device address.
    {CC1101_PKTLEN  , 0xFF}  //  Packet length.
};                            
*/
static const uint8_t paTable[] = PA_TABLE;

/*
================================================================================
Function : CC1101ReadReg( )
           read a byte from the specified register
INPUT    : addr, The address of the register
OUTPUT   : the byte read from the register
================================================================================
*/
uint8_t CC1101ReadReg( uint8_t addr )
{
    uint8_t i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_SINGLE);
    i = SPI_ExchangeByte( 0xFF );
    CC_CSN_HIGH( );
    return i;
}

/*
================================================================================
Function : CC1101ReadMultiReg( )
    Read some bytes from the registers continously
INPUT    : addr, The address of the register
           buff, The buffer stores the data
           size, How many bytes should be read
OUTPUT   : None
================================================================================
*/
void CC1101ReadMultiReg( uint8_t addr, uint8_t* buff, uint8_t size )
{
    uint8_t i, j;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_BURST);
    for( i = 0; i < size; i ++ )
    {
        for( j = 0; j < 20; j ++ );
        *( buff + i ) = SPI_ExchangeByte( 0xFF );
    }
    CC_CSN_HIGH( );
}
/*
================================================================================
Function : CC1101ReadStatus( )
    Read a status register
INPUT    : addr, The address of the register
OUTPUT   : the value read from the status register
================================================================================
*/
uint8_t CC1101ReadStatus( uint8_t addr )
{
    uint8_t i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_BURST);
    i = SPI_ExchangeByte( 0xFF );
    CC_CSN_HIGH( );
    return i;
}

/*
================================================================================
Function : CC1101SetTRMode( )
    Set the device as TX mode or RX mode
INPUT    : mode selection
OUTPUT   : None
================================================================================
*/
/*
void CC1101SetTRMode( TRMODE mode )
{
    if( mode == TX_MODE )
    {
        CC1101WriteReg(CC1101_IOCFG0,0x46);
        CC1101WriteCmd( CC1101_STX );
    }
    else if( mode == RX_MODE )
    {
        CC1101WriteReg(CC1101_IOCFG0,0x46);
        CC1101WriteCmd( CC1101_SRX );
    }
}
*/
/*
================================================================================
Function : CC1101WriteReg( )
    Write a byte to the specified register
INPUT    : addr, The address of the register
           value, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteReg( uint8_t addr, uint8_t value )
{
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr );
    SPI_ExchangeByte( value );
    CC_CSN_HIGH( );
}
/*
================================================================================
Function : CC1101WriteMultiReg( )
    Write some bytes to the specified register
INPUT    : addr, The address of the register
           buff, a buffer stores the values
           size, How many byte should be written
OUTPUT   : None
================================================================================
*/
void CC1101WriteMultiReg( uint8_t addr, uint8_t* buff, uint8_t size )
{
    uint8_t i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | WRITE_BURST );
    for( i = 0; i < size; i ++ )
    {
        SPI_ExchangeByte( *( buff + i ) );
    }
    CC_CSN_HIGH( );
}

/*
================================================================================
Function : CC1101WriteCmd( )
    Write a command byte to the device
INPUT    : command, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteCmd( uint8_t command )
{
    CC_CSN_LOW( );
    SPI_ExchangeByte( command );
    CC_CSN_HIGH( );
}

/*
================================================================================
Function : CC1101ClrTXBuff( )
    Flush the TX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
void CC1101ClrTXBuff( void )
{
    CC1101WriteCmd(CC1101_SIDLE);
    CC1101WriteCmd( CC1101_SFTX );
}
================================================================================
Function : CC1101ClrRXBuff( )
    Flush the RX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
void CC1101ClrRXBuff( void )
{
    CC1101WriteCmd(CC1101_SIDLE);
    CC1101WriteCmd( CC1101_SFRX );
}
================================================================================
Function : CC1101SetAddress( )
    Set the address and address mode of the CC1101
INPUT    : address, The address byte
           AddressMode, the address check mode
OUTPUT   : None
================================================================================
void CC1101SetAddress( uint8_t address, AddressMode addressMode)
{
    uint8_t btmp;
    
    btmp = CC1101ReadReg( CC1101_PKTCTRL1 ) & ~0x03;
    CC1101WriteReg(CC1101_PKTCTRL1, btmp | (uint8_t)addressMode);
    CC1101WriteReg(CC1101_ADDR, address);
    //if     ( AddressMode == BROAD_ALL )     {}
    //else if( AddressMode == BROAD_NO  )     { btmp |= 0x01; }
    //else if( AddressMode == BROAD_0   )     { btmp |= 0x02; }
    //else if( addressMode == BROAD_0AND255 ) { btmp |= 0x03; }       
}
*/

/*
================================================================================
Function : CC1101GetRXCnt( )
    Get received count of CC1101
INPUT    : None
OUTPUT   : How many bytes have been received
================================================================================
*/
uint8_t CC1101GetRXCnt(void)
{
    uint8_t rxbytes;
    // Read until two readings match.  Silicon error workaround
    do {
        rxbytes = CC1101ReadStatus( CC1101_RXBYTES );
    } while (rxbytes != CC1101ReadStatus(CC1101_RXBYTES));

    return (rxbytes & BYTES_IN_RXFIFO_MSK);
}
/*
================================================================================
Function : CC1101SetSYNC( )
    Set the SYNC bytes of the CC1101
INPUT    : sync, 16bit sync 
OUTPUT   : None
================================================================================
*/
void CC1101SetSYNC( uint16_t sync )
{
    CC1101WriteReg(CC1101_SYNC1, 0xFF & ( sync>>8 ) );
    CC1101WriteReg(CC1101_SYNC0, 0xFF & sync ); 
}

/*
================================================================================
Function : CC1101WriteTxFifo( )
    Fill a packet
INPUT    : dstAddress  - destination address,
           txBuffer - the buffer stores data to be sent
           size - how many bytes should be sent
OUTPUT   : None
================================================================================
*/
/*
void CC1101WriteTxFifo(uint8_t dstAddress, uint8_t* txBuffer, uint8_t size)
{
    //uint8_t address;
    //static uint8_t flag = 0;
	
    //if(mode == BROADCAST)          address = 0;
    //else if(mode == ADDRESS_CHECK) address = CC1101ReadReg(CC1101_ADDR);

    //if(flag == 0)   
    //{
    //    printf("local_address:%d\r\n", (int)address);
    //    flag = 1;
    //}
    //CC1101ClrTXBuff();
    
    // Clear tx FIFO
    CC1101WriteCmd(CC1101_SIDLE);
    CC1101WriteCmd( CC1101_SFTX );
    
    //if((CC1101ReadReg(CC1101_PKTCTRL1)& ~0x03)!= 0)
    //{ 
    //    address = RX_Address;
        CC1101WriteReg(CC1101_TXFIFO, size + 1);
        CC1101WriteReg(CC1101_TXFIFO, dstAddress);
    //}
    //else
    //{
    //    CC1101WriteReg(CC1101_TXFIFO, size);
    //}

    CC1101WriteMultiReg(CC1101_TXFIFO, txbuffer, size);
    //CC1101SetTRMode(TX_MODE);
    //while(CC_IRQ_READ()!= 0);
    //while(CC_IRQ_READ()== 0);

    //CC1101ClrTXBuff();
}
void CC1101WriteTxFifo(uint8_t* txBuffer, uint8_t size)
{
    
    // Clear tx FIFO
    //CC1101WriteCmd(CC1101_SIDLE);
    //CC1101WriteCmd( CC1101_SFTX );
    
    // Write lenght byte    
    CC1101WriteReg(CC1101_TXFIFO, size);
    
    // Write data
    CC1101WriteMultiReg(CC1101_TXFIFO, txBuffer, size);
}
*/

/*
================================================================================
Function : CC1101RecPacket( )
    Receive a packet
INPUT    : rxBuffer, A buffer store the received data
OUTPUT   :  0:no data, if received count > rxBufSize, then there is part of data in rxBuffer
================================================================================
*/
uint8_t CC1101ReadRxFifo(uint8_t* rxBuffer, uint8_t rxBufSize)
{
    uint8_t status[2], pktLen, len;

    pktLen = 0; // No rx data, filtered out by length or address or overflow
    
    //if(CC1101GetRXCnt()!= 0)
    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
    if (CC1101ReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO_MSK) {
        pktLen = CC1101ReadReg(CC1101_RXFIFO); // Read length byte
        //if((CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03)!= 0)
        //{
        //    CC1101ReadReg(CC1101_RXFIFO);
        //    pktLen --; //Mu
        //}
        len = pktLen;
        if(pktLen > rxBufSize)
            len = rxBufSize;
        
        CC1101ReadMultiReg(CC1101_RXFIFO, rxBuffer, len); // Pull data
        CC1101ReadMultiReg(CC1101_RXFIFO, status, 2);     // Read  status bytes

        //CC1101ClrRXBuff();
        
        if( !(status[1] & CRC_OK) )
            pktLen = 0;           // Crc error
    }
    
    // Clear Rx Fifo
    CC1101WriteCmd(CC1101_SIDLE); // Mandatory, because it can be Rx overflow state
    CC1101WriteCmd( CC1101_SFRX );
    
    return pktLen; 
}

/*
================================================================================
Function : CC1101WORInit( )
    Initialize the WOR function of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void  CC1101WORInit(void)
{
    // Main Radio Control State Machine Configuration 
    //[5:4]=11,When to perform automatic calibration = Every 4th time When going from IDLE to RX or TX
    //[3:2]=10,Timeout after XOSC start = Approx. 149 – 155 us 
    //[1]=0,Disables the pin radio control option 
    //[0]=0,Force the XOSC to stay on in the SLEEP state =  Disable
    CC1101WriteReg(CC1101_MCSM0,0x38); // 18 was 38

    // Wake On Radio Control 
    //[7]=0,Power down signal to RC oscillator.When written to 0,automatic initial calibration will be performed. 
    //[6:4]=111,Event 1 timeout = (1.333 – 1.385 ms) ,clock periods after Event 0 before Event 1 times out. 
    //[3]=1,Enables the RC oscillator calibration. 
    //[1:0]=00,WOR_RES =00,Controls the Event 0 resolution as well as maximum timeout of the WOR module 
    //and maximum timeout under normal RX operation: 
    CC1101WriteReg(CC1101_WORCTRL,0x78);  //78  0x38 
    
    //// Degrade sensitivity by increase of the RSSI threshold 
    //CC1101WriteReg(CC1101_AGCCTRL1, CC1101ReadReg(CC1101_AGCCTRL1) | 0x07);
    
    // Set preamble quality threshould (PQT) =1(bytes?)
    // In the register was: append status = 1, address check mode = 10b - native and 0 broad addr check
    CC1101WriteReg(CC1101_PKTCTRL1, CC1101ReadReg(CC1101_PKTCTRL1) | 0x60); // was 20

    //Main Radio Control State Machine Configuration 
    //[4]=1,Direct RX termination, based on RSSI measurement, enabled
    //[3]=1,When the RX_TIME timer expires, the chip checks if sync word is
    //      found  or PQI is set  when RX_TIME_QUAL=1;
    //[2:0]=011,Timeout for sync word search in RX for both WOR mode and normal RX 
    //operation. The timeout is relative to the programmed EVENT0 timeout.  
    //RX timeout (when WOR_RES = 0) = 1.563% * 75ms = 1.17ms 
    CC1101WriteReg(CC1101_MCSM2,0x1a);  //was 1b

    //EVENT0 timeout = 75ms, Event0 = 26000000Hz * 0.075s/750 = 2600    
    CC1101WriteReg(CC1101_WOREVT1,0x0a); // High Byte Event0
    CC1101WriteReg(CC1101_WOREVT0,0x28); // Low Byte Event0 

    //Reset real time clock to Event1 value. 
    //CC1101WriteCmd( CC1101_SWORRST );
}


/*
================================================================================
Function : CC1101Reset( )
    Reset the CC1101 device
INPUT    : None
OUTPUT   : None
================================================================================
*/
static void CC1101Reset( void )
{
    uint8_t x;

    CC_CSN_HIGH( );
    CC_CSN_LOW( );
    CC_CSN_HIGH( );
    for( x = 0; x < 250; x ++ );        // 100 40us
    CC1101WriteCmd(CC1101_SRES);
}

/*
================================================================================
Function : CC1101Init( )
    Initialize the CC1101, User can modify it
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Init(uint8_t maxPacketLength, uint8_t selfAddress, uint8_t numPreamble)
{
    uint8_t i;
    
    // Very first, because CS pin initialized here. 
    BSP_Rf_Init();
    
    CC1101Reset();    
    
    for(i = 0; i < sizeof(rfSetting)/sizeof(RfSetting); i++) {
        CC1101WriteReg(rfSetting[i].registerAddress, rfSetting[i].registerValue);
    }
 
    //for(i = 0; i < 0x2f; i++) {
    //    j = CC1101ReadReg(i);
    //    printf("%d  ", (int)j);
    //}
    
    //static uint8_t regArray[0x2f];
    //for(i = 0; i < 0x2f; i++) {
    //    regArray[i] = CC1101ReadReg(i);
    //    regArray[i] = regArray[i];
    //}
    
    //CC1101SetAddress(TX_Address, BROAD_0AND255);  // ´Ó»úµØÖ·
    
    // Set self address 
    CC1101WriteReg(CC1101_ADDR, selfAddress);
    // Set maximum received packet length allowed 
    CC1101WriteReg(CC1101_PKTLEN, maxPacketLength);
    
    CC1101SetSYNC(RF_DEFAULT_SYNC_WORD);           // 8799
    
    // Set preamble length
    i = CC1101ReadReg(CC1101_MDMCFG1);
    i &= ~0x70;
    i |= (numPreamble << 4);
    CC1101WriteReg(CC1101_MDMCFG1, i); //0x42; was default 0x22 8 bytes preamble
    
    //CC1101WriteReg(CC1101_MDMCFG1, 0x22); // Modem Configuration: chan spacing = 200 kHz,      
    //CC1101WriteReg(CC1101_MDMCFG0, 0xF8); // num preamble=4 bytes, FEC disable      

    CC1101WriteMultiReg(CC1101_PATABLE, (uint8_t*)paTable, 8);
    
    // Init wake on radio receive mode
    CC1101WORInit();
    //CC1101WriteReg(CC1101_MCSM0,0x38);
    ////CC1101WriteReg(CC1101_MCSM2,0x07);
   //CC1101WriteReg(CC1101_PKTCTRL1, CC1101ReadReg(CC1101_PKTCTRL1) | 0x60); //3 bytes or bits? PQT threshold 
    //CC1101WriteReg(CC1101_IOCFG0,0x08); // Interrupt on PQT threshold reached
    //CC1101WriteReg(CC1101_IOCFG2,0x24); // Interrupt on WOR EVENT0
    //CC1101WriteReg(CC1101_IOCFG2,0x26); // Interrupt on WOR EVENT1
    
    
    i = CC1101ReadStatus(CC1101_PARTNUM);//for test, must be 0x80 00?
    i = CC1101ReadStatus(CC1101_VERSION);//for test, VERSION = 0x14

    // Set power down state
    CC1101WriteCmd(CC1101_SPWD);
}






void  CC1101WORDeInit(void)
{
    // Main Radio Control State Machine Configuration 
    //[5:4]=01,When to perform automatic calibration = Every time When going from IDLE to RX or TX
    //[3:2]=10,Timeout after XOSC start = Approx. 149 – 155 us 
    //[1]=0,Disables the pin radio control option 
    //[0]=0,Force the XOSC to stay on in the SLEEP state =  Disable
    CC1101WriteReg(CC1101_MCSM0,0x18); 

    // Wake On Radio Control 
    //[7]=1,Power down signal to RC oscillator.When written to 0,automatic initial calibration will be performed. 
    //[6:4]=111,Event 1 timeout = (1.333 – 1.385 ms) ,clock periods after Event 0 before Event 1 times out. 
    //[3]=1,Enables the RC oscillator calibration. 
    //[1:0]=11,WOR_RES =11,Controls the Event 0 resolution as well as maximum timeout of the WOR module 
    //and maximum timeout under normal RX operation: 
    CC1101WriteReg(CC1101_WORCTRL,0xfb);  
    
    //// Degrade sensitivity by increase of the RSSI threshold 
    //CC1101WriteReg(CC1101_AGCCTRL1, CC1101ReadReg(CC1101_AGCCTRL1) | 0x07);
    
    // Disable preamble quality threshould (PQT).
    // In the register was: append status = 1, address check mode = 10b - native and 0 broad addr check
    CC1101WriteReg(CC1101_PKTCTRL1, CC1101ReadReg(CC1101_PKTCTRL1) & ~0xe0); 

    //Main Radio Control State Machine Configuration 
    //[4]=0,Direct RX termination, based on RSSI measurement, enabled
    //[3]=0,When the RX_TIME timer expires, the chip checks if sync word is
    //      found  or PQI is set  when RX_TIME_QUAL=1;
    //[2:0]=111,Timeout for sync word search in RX for both WOR mode and normal RX 
    //operation. The timeout is disabled.  
    //RX timeout (when WOR_RES = 0) = 1.563% * 75ms = 1.17ms 
    CC1101WriteReg(CC1101_MCSM2,0x08); 

    //EVENT0 timeout = 75ms, Event0 = 26000000Hz * 0.075s/750 = 2600    
    //CC1101WriteReg(CC1101_WOREVT1,0x0a); // High Byte Event0
    //CC1101WriteReg(CC1101_WOREVT0,0x28); // Low Byte Event0 

    //Reset real time clock to Event1 value. 
    //CC1101WriteCmd( CC1101_SWORRST );
}

/*
================================================================================
------------------------------------THE END-------------------------------------
================================================================================
*/
