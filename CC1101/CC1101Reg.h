/**
  ******************************************************************************
  * @file    Keyfob/CC1101Reg.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   CC1101 registers header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __CC1101REG_H
#define __CC1101REG_H

//***************************************************************************
// CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CC1101_IOCFG2       0x00    // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01    // GDO1 output pin configuration
#define CC1101_IOCFG0       0x02    // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03    // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04    // Sync word, high INT8U
#define CC1101_SYNC0        0x05    // Sync word, low INT8U
#define CC1101_PKTLEN       0x06    // Packet length
#define CC1101_PKTCTRL1     0x07    // Packet automation control
#define CC1101_PKTCTRL0     0x08    // Packet automation control
#define CC1101_ADDR         0x09    // Device address
#define CC1101_CHANNR       0x0A    // Channel number
#define CC1101_FSCTRL1      0x0B    // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C    // Frequency synthesizer control
#define CC1101_FREQ2        0x0D    // Frequency control word, high INT8U
#define CC1101_FREQ1        0x0E    // Frequency control word, middle INT8U
#define CC1101_FREQ0        0x0F    // Frequency control word, low INT8U
#define CC1101_MDMCFG4      0x10    // Modem configuration
#define CC1101_MDMCFG3      0x11    // Modem configuration
#define CC1101_MDMCFG2      0x12    // Modem configuration
#define CC1101_MDMCFG1      0x13    // Modem configuration
#define CC1101_MDMCFG0      0x14    // Modem configuration
#define CC1101_DEVIATN      0x15    // Modem deviation setting
#define CC1101_MCSM2        0x16    // Main Radio Control State Machine configuration
#define CC1101_MCSM1        0x17    // Main Radio Control State Machine configuration
#define CC1101_MCSM0        0x18    // Main Radio Control State Machine configuration
#define CC1101_FOCCFG       0x19    // Frequency Offset Compensation configuration
#define CC1101_BSCFG        0x1A    // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B    // AGC control
#define CC1101_AGCCTRL1     0x1C    // AGC control
#define CC1101_AGCCTRL0     0x1D    // AGC control
#define CC1101_WOREVT1      0x1E    // High INT8U Event 0 timeout
#define CC1101_WOREVT0      0x1F    // Low INT8U Event 0 timeout
#define CC1101_WORCTRL      0x20    // Wake On Radio control
#define CC1101_FREND1       0x21    // Front end RX configuration
#define CC1101_FREND0       0x22    // Front end TX configuration
#define CC1101_FSCAL3       0x23    // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24    // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25    // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26    // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27    // RC oscillator configuration
#define CC1101_RCCTRL0      0x28    // RC oscillator configuration
#define CC1101_FSTEST       0x29    // Frequency synthesizer calibration control
#define CC1101_PTEST        0x2A    // Production test
#define CC1101_AGCTEST      0x2B    // AGC test
#define CC1101_TEST2        0x2C    // Various test settings
#define CC1101_TEST1        0x2D    // Various test settings
#define CC1101_TEST0        0x2E    // Various test settings

// Strobe commands
#define CC1101_SRES         0x30    // Reset chip.
#define CC1101_SFSTXON      0x31    // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                    // If in RX/TX: Go to a wait state where only the synthesizer is
                                    // running (for quick RX / TX turnaround).
#define CC1101_SXOFF        0x32    // Turn off crystal oscillator.
#define CC1101_SCAL         0x33    // Calibrate frequency synthesizer and turn it off
                                    // (enables quick start).
#define CC1101_SRX          0x34    // Enable RX. Perform calibration first if coming from IDLE and
                                    // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX          0x35    // In IDLE state: Enable TX. Perform calibration first if
                                    // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                    // Only go to TX if channel is clear.
#define CC1101_SIDLE        0x36    // Exit RX / TX, turn off frequency synthesizer and exit
                                    // Wake-On-Radio mode if applicable.
#define CC1101_SAFC         0x37    // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR         0x38    // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD         0x39    // Enter power down mode when CSn goes high.
#define CC1101_SFRX         0x3A    // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B    // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C    // Reset real time clock.
#define CC1101_SNOP         0x3D    // No operation. May be used to pad strobe commands to two
                                    // INT8Us for simpler software.
                                    
#define CC1101_PARTNUM      0x30
#define CC1101_VERSION      0x31
#define CC1101_FREQEST      0x32
#define CC1101_LQI          0x33
#define CC1101_RSSI         0x34
//Value      State name         State
//0            SLEEP            SLEEP 
//1            IDLE             IDLE
//2            XOFF             XOFF
//3           VCOON_MC          MANCAL 
//4           REGON_MC          MANCAL 
//5           MANCAL            MANCAL 
//6           VCOON             FS_WAKEUP 
//7           REGON             FS_WAKEUP 
//8          STARTCAL           CALIBRATE 
//9          BWBOOST            SETTLING 
//10         FS_LOCK            SETTLING 
//11         IFADCON            SETTLING 
//12         ENDCAL             CALIBRATE
//13          RX                RX 
//14         RX_END             RX 
//15         RX_RST             RX 
//16         TXRX_SWITCH        TXRX_SETTLING 
//17        RXFIFO_OVERFLOW     RXFIFO_OVERFLOW
//18         FSTXON             FSTXON 
//19          TX                TX 
//20         TX_END             TX 
//21         RXTX_SWITCH        RXTX_SETTLING 
//22      TXFIFO_UNDERFLOW      TXFIFO_UNDERFLOW 
#define CC1101_MARCSTATE    0x35
#define CC1101_WORTIME1     0x36
#define CC1101_WORTIME0     0x37
//[7]  CRC_OK         The last CRC comparison matched. Cleared when entering/restarting RX mode. 
//[6]   CS            Carrier sense. Cleared when entering IDLE mode. 
//[5]  PQT_REACHED    Preamble Quality reached.  If leaving RX state when this bit is set it will 
                      //remain asserted until the chip re-enters RX state (MARCSTATE=0x0D). The 
                      //bit will also be cleared if PQI goes below the programmed PQT value
//[4]   CCA           Channel is clear 
//[3]   SFD           Start of Frame Delimiter.  In RX, this bit is asserted when sync word has 
                      //been received and de-asserted at the end of the packet. It will also de-
                      //assert when a packet is discarded due to address or maximum length 
                      //filtering or the radio enters RXFIFO_OVERFLOW state. In TX this bit will 
                      //always read as 0.
//[2]   GDO2          Current GDO2 value.Note: the reading gives the non-inverted value 
                      //irrespective of what IOCFG2.GDO2_INV is programmed to. 
                      //It is not recommended to check for PLL lock by reading PKTSTATUS[2] 
                      //with GDO2_CFG=0x0A. 
//[0]   GDO0          Current GDO0 value. Note: the reading gives the non-inverted value 
                      //irrespective of what IOCFG0.GDO0_INV is programmed to.   
                      //It is not recommended to check for PLL lock by reading PKTSTATUS[0] 
                      //with GDO0_CFG=0x0A. 
#define CC1101_PKTSTATUS    0x38
#define CC1101_VCO_VC_DAC   0x39
#define CC1101_TXBYTES      0x3A
#define CC1101_RXBYTES      0x3B

#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F

#define WRITE_BURST         0x40    // 
#define READ_SINGLE         0x80    // 
#define READ_BURST          0xC0    // 
#define BYTES_IN_RXFIFO_MSK 0x7F    // 
#define CRC_OK              0x80    // 

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/