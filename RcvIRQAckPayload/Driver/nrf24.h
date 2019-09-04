#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"


// SPI(nRF24L01) commands
#define NRF24_READ_REG                  (uint8_t)0x00  // b000A AAAA // Define read command to register Read command and status registers. AAAAA = 5 bit Register Map Address 
#define NRF24_WRITE_REG                 (uint8_t)0x20  // b001A AAAA  // Define write command to register AAAAA = 5 bit Register Map Address 
#define NRF24_RD_RX_PLOAD               (uint8_t)0x61  // Define RX payload register address
#define NRF24_WR_TX_PLOAD               (uint8_t)0xA0  // Define TX payload register address
#define NRF24_FLUSH_TX                  (uint8_t)0xE1  // Define flush TX register command
#define NRF24_FLUSH_RX                  (uint8_t)0xE2  // Define flush RX register command
#define NRF24_REUSE_TX_PL               (uint8_t)0xE3  // Define reuse TX payload register command
#define NRF24_ACTIVATE                  (uint8_t)0x50
#define NRF24_R_RX_PL_WID               (uint8_t)0x60    //Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO. 
#define NRF24_W_ACK_PAYLOAD             (uint8_t)0xA8  // b1010 1PPP // Write Payload to be transmitted together with ACK packet on PIPE PPP
#define NRF24_W_TX_PAYLOAD_NO_ACK       (uint8_t)0xB0    // Used in TX mode. Disables AUTOACK on this specific packet. 
#define NRF24_NOP                       (uint8_t)0xFF  // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses)
#define NRF24_CONFIG          (uint8_t)0x00  // 'Config' register address
#define NRF24_EN_AA           (uint8_t)0x01  // 'Enable Auto Acknowledgment' register address
#define NRF24_EN_RXADDR       (uint8_t)0x02  // 'Enabled RX addresses' register address
#define NRF24_SETUP_AW        (uint8_t)0x03  // 'Setup address width' register address
#define NRF24_SETUP_RETR      (uint8_t)0x04  // 'Setup Auto. Retrans' register address
#define NRF24_RF_CH           (uint8_t)0x05  // 'RF channel' register address
#define NRF24_RF_SETUP        (uint8_t)0x06  // 'RF setup' register address
#define NRF24_STATUS          (uint8_t)0x07  // 'Status' register address
#define NRF24_OBSERVE_TX      (uint8_t)0x08  // 'Observe TX' register address
#define NRF24_CD              (uint8_t)0x09  // 'Carrier Detect' register address
#define NRF24_RX_ADDR_P0      (uint8_t)0x0A  // 'RX address pipe0' register address
#define NRF24_RX_ADDR_P1      (uint8_t)0x0B  // 'RX address pipe1' register address
#define NRF24_RX_ADDR_P2      (uint8_t)0x0C  // 'RX address pipe2' register address
#define NRF24_RX_ADDR_P3      (uint8_t)0x0D  // 'RX address pipe3' register address
#define NRF24_RX_ADDR_P4      (uint8_t)0x0E  // 'RX address pipe4' register address
#define NRF24_RX_ADDR_P5      (uint8_t)0x0F  // 'RX address pipe5' register address
#define NRF24_TX_ADDR         (uint8_t)0x10  // 'TX address' register address
#define NRF24_RX_PW_P0        (uint8_t)0x11  // 'RX payload width, pipe0' register address
#define NRF24_RX_PW_P1        (uint8_t)0x12  // 'RX payload width, pipe1' register address
#define NRF24_RX_PW_P2        (uint8_t)0x13  // 'RX payload width, pipe2' register address
#define NRF24_RX_PW_P3        (uint8_t)0x14  // 'RX payload width, pipe3' register address
#define NRF24_RX_PW_P4        (uint8_t)0x15  // 'RX payload width, pipe4' register address
#define NRF24_RX_PW_P5        (uint8_t)0x16  // 'RX payload width, pipe5' register address
#define NRF24_FIFO_STATUS     (uint8_t)0x17  // 'FIFO Status Register' register address
//// In datasheet this 3 register memory with address N/A
// #define NRF24_ACK_PLD
// #define NRF24_TX_PLD
// #define NRF24_RX_PLD
#define NRF24_DYNPD           (uint8_t)0x1C
#define NRF24_FEATURE         (uint8_t)0x1D

// define for BK2421
// for BK2421 described Register Bank 1

#define NRF24_RB1_00					(uint8_t)0x00 // Must write with 0x404B01E2
#define NRF24_RB1_01					(uint8_t)0x01 // Must write with 0xC04B0000
#define NRF24_RB1_02					(uint8_t)0x02 // Must write with 0xD0FC8C02
#define NRF24_RB1_03					(uint8_t)0x03 // Must write with 0x99003941
// for Register 4
// bits 31:0 W Must write with 0xD99E860B(High Power) For single carrier mode:0xD99E8621
// bits 	20 W RF output power in TX mode: 0:Low power(-30dB down) 1:High power
#define NRF24_RB1_04					(uint8_t)0x04 
// for register 5
// bits 31:0 W	Must write with 0x24067FA6(Disable RSSI) RSSI_TH 29:26 W RSSI Threshold for CD detect 0: -97 dBm, 2 dB/step, 15: -67 dBm
// bit 		18 W	RSSI_EN 18 0 W RSSI measurement: 0:Enable 1:Disable
#define NRF24_RB1_05					(uint8_t)0x05 // Must write with 0x404B01E2
#define NRF24_RB1_CHIP_ID			(uint8_t)0x08 // BEKEN Chip ID: 0x00000063(BK2421) 
#define NRF24_RB1_OC					(uint8_t)0x0C // Please initialize with 0x00731200
#define NRF24_RB1_NEW_FEATURE	(uint8_t)0x0D // Please initialize with 0x0080B436
#define NRF24_RB1_RAMP				(uint8_t)0x0E // Please write with 0xFFFFFEF7CF208104082041

#define	NRF24_SWITCH_BANK			(uint8_t)0x53	// for BL2421

// define bits in register
// status register
#define NRF24_MASK_RX_DR	(uint8_t)0x40
#define NRF24_MASK_TX_DS	(uint8_t)0x20
#define NRF24_MASK_MAX_RT			(uint8_t)0x10
#define NRF24_ENABLE_CRC	(uint8_t)0x08
#define NRF24_CRCO				(uint8_t)0x04
#define NRF24_CONFIG_PWR_UP			(uint8_t)0x02
#define NRF24_CONFIG_PRIM_RX			(uint8_t)0x01

// Register bits definitions
#define NRF24_CONFIG_PRIM_RX       (uint8_t)0x01 // PRIM_RX bit in CONFIG register
#define NRF24_CONFIG_PWR_UP        (uint8_t)0x02 // PWR_UP bit in CONFIG register
#define NRF24_FLAG_RX_DR           (uint8_t)0x40 // RX_DR bit (data ready RX FIFO interrupt)
#define NRF24_FLAG_TX_DS           (uint8_t)0x20 // TX_DS bit (data sent TX FIFO interrupt)
#define NRF24_FLAG_MAX_RT          (uint8_t)0x10 // MAX_RT bit (maximum number of TX retransmits interrupt)

// Register masks definitions
#define NRF24_MASK_REG_MAP         (uint8_t)0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
#define NRF24_MASK_CRC             (uint8_t)0x0C // Mask for CRC bits [3:2] in CONFIG register
#define NRF24_MASK_STATUS_IRQ_RX   (uint8_t)0x40 
#define NRF24_MASK_STATUS_IRQ_TX   (uint8_t)0x20 
#define NRF24_MASK_STATUS_IRQ_RT   (uint8_t)0x10 
#define NRF24_MASK_STATUS_IRQ      (uint8_t)0x70 // Mask for all IRQ bits in STATUS register
#define NRF24_MASK_RF_PWR          (uint8_t)0x06 // Mask RF_PWR[2:1] bits in RF_SETUP register
#define NRF24_MASK_RX_P_NO         (uint8_t)0x0E // Mask RX_P_NO[3:1] bits in STATUS register
#define NRF24_MASK_DATARATE        (uint8_t)0x28 // Mask RD_DR_[5,3] bits in RF_SETUP register
#define NRF24_MASK_EN_RX           (uint8_t)0x3F // Mask ERX_P[5:0] bits in EN_RXADDR register
#define NRF24_MASK_RX_PW           (uint8_t)0x3F // Mask [5:0] bits in RX_PW_Px register
#define NRF24_MASK_RETR_ARD        (uint8_t)0xF0 // Mask for ARD[7:4] bits in SETUP_RETR register
#define NRF24_MASK_RETR_ARC        (uint8_t)0x0F // Mask for ARC[3:0] bits in SETUP_RETR register
#define NRF24_MASK_RXFIFO          (uint8_t)0x03 // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
#define NRF24_MASK_TXFIFO          (uint8_t)0x30 // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
#define NRF24_MASK_PLOS_CNT        (uint8_t)0xF0 // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
#define NRF24_MASK_ARC_CNT         (uint8_t)0x0F // Mask for ARC_CNT[3:0] bits in OBSERVE_TX register

#define NRF24_MASK_EN_DPL						(uint8_t)0x04	// Mask for Enables Dynamics Payload Length
#define NRF24_MASK_EN_ACK_PAY				(uint8_t)0x02	// Mask for Enables Payload with ACK
#define NRF24_MASK_EN_DYN_ACK				(uint8_t)0x01	// Mask for Enables the W_TX_PAYLOAD_NOACK command


#define NRF24_CE_LOW()	GPIO_ResetBits(NRF24_PORT, NRF24_PIN_CE)
#define NRF24_CE_HIGH()	GPIO_SetBits(NRF24_PORT, NRF24_PIN_CE)

#define NRF24_CSN_LOW()		GPIO_ResetBits(NRF24_PORT, NRF24_PIN_CSN)
#define NRF24_CSN_HIGH()	GPIO_SetBits(NRF24_PORT, NRF24_PIN_CSN)

#define NRF24_TX_ADDRESS_WIDTH    (uint8_t)5   // 5 bytes TX(RX) address width
#define NRF24_TX_PAYLOAD_WIDTH  (uint8_t)2  // 20 bytes TX payload

#define NRF24_TEST_ADDR "Podar"

// Retransmit delay
enum {
	NRF24_ARD_NONE   = (uint8_t)0x00, // Dummy value for case when retransmission is not used
	NRF24_ARD_250us  = (uint8_t)0x00,
	NRF24_ARD_500us  = (uint8_t)0x01,
	NRF24_ARD_750us  = (uint8_t)0x02,
	NRF24_ARD_1000us = (uint8_t)0x03,
	NRF24_ARD_1250us = (uint8_t)0x04,
	NRF24_ARD_1500us = (uint8_t)0x05,
	NRF24_ARD_1750us = (uint8_t)0x06,
	NRF24_ARD_2000us = (uint8_t)0x07,
	NRF24_ARD_2250us = (uint8_t)0x08,
	NRF24_ARD_2500us = (uint8_t)0x09,
	NRF24_ARD_2750us = (uint8_t)0x0A,
	NRF24_ARD_3000us = (uint8_t)0x0B,
	NRF24_ARD_3250us = (uint8_t)0x0C,
	NRF24_ARD_3500us = (uint8_t)0x0D,
	NRF24_ARD_3750us = (uint8_t)0x0E,
	NRF24_ARD_4000us = (uint8_t)0x0F
};

// Data rate
enum {
	NRF24_DR_250kbps = (uint8_t)0x20, // 250kbps data rate
	NRF24_DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
	NRF24_DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
};

// RF output power in TX mode
enum {
	NRF24_TXPWR_18dBm = (uint8_t)0x00, // -18dBm
	NRF24_TXPWR_12dBm = (uint8_t)0x02, // -12dBm
	NRF24_TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
	NRF24_TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
};

enum {
	NRF24_CRC_off   = (uint8_t)0x00, // CRC disabled
	NRF24_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
	NRF24_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
};

// nRF24L01 power control
enum {
	NRF24_PWR_UP   = (uint8_t)0x02, // Power up
	NRF24_PWR_DOWN = (uint8_t)0x00  // Power down
};

// Transceiver mode
enum {
	NRF24_MODE_RX = (uint8_t)0x01, // PRX
	NRF24_MODE_TX = (uint8_t)0x00  // PTX
};

// Enumeration of RX pipe addresses and TX address
enum {
	NRF24_PIPE0  = (uint8_t)0x00, // pipe0
	NRF24_PIPE1  = (uint8_t)0x01, // pipe1
	NRF24_PIPE2  = (uint8_t)0x02, // pipe2
	NRF24_PIPE3  = (uint8_t)0x03, // pipe3
	NRF24_PIPE4  = (uint8_t)0x04, // pipe4
	NRF24_PIPE5  = (uint8_t)0x05, // pipe5
	NRF24_PIPETX = (uint8_t)0x06  // TX address (not a pipe in fact)
};

// State of auto acknowledgment for specified pipe
enum {
	NRF24_AA_OFF = (uint8_t)0x00,
	NRF24_AA_ON  = (uint8_t)0x01
};

// Status of the RX FIFO
enum {
	NRF24_STATUS_RXFIFO_DATA  = (uint8_t)0x00, // The RX FIFO contains data and available locations
	NRF24_STATUS_RXFIFO_EMPTY = (uint8_t)0x01, // The RX FIFO is empty
	NRF24_STATUS_RXFIFO_FULL  = (uint8_t)0x02, // The RX FIFO is full
	NRF24_STATUS_RXFIFO_ERROR = (uint8_t)0x03  // Impossible state: RX FIFO cannot be empty and full at the same time
};

// Status of the TX FIFO
enum {
	NRF24_STATUS_TXFIFO_DATA  = (uint8_t)0x00, // The TX FIFO contains data and available locations
	NRF24_STATUS_TXFIFO_EMPTY = (uint8_t)0x01, // The TX FIFO is empty
	NRF24_STATUS_TXFIFO_FULL  = (uint8_t)0x02, // The TX FIFO is full
	NRF24_STATUS_TXFIFO_ERROR = (uint8_t)0x03  // Impossible state: TX FIFO cannot be empty and full at the same time
};

// Result of RX FIFO reading
typedef enum {
	NRF24_RX_PIPE0  = (uint8_t)0x00, // Packet received from the PIPE#0
	NRF24_RX_PIPE1  = (uint8_t)0x01, // Packet received from the PIPE#1
	NRF24_RX_PIPE2  = (uint8_t)0x02, // Packet received from the PIPE#2
	NRF24_RX_PIPE3  = (uint8_t)0x03, // Packet received from the PIPE#3
	NRF24_RX_PIPE4  = (uint8_t)0x04, // Packet received from the PIPE#4
	NRF24_RX_PIPE5  = (uint8_t)0x05, // Packet received from the PIPE#5
	NRF24_RX_EMPTY  = (uint8_t)0xff  // The RX FIFO is empty
} NRF24_RXResult;

// Addresses of the RX_PW_P# registers
static const uint8_t NRF24_RX_PW_PIPE[6] = {
		NRF24_RX_PW_P0,
		NRF24_RX_PW_P1,
		NRF24_RX_PW_P2,
		NRF24_RX_PW_P3,
		NRF24_RX_PW_P4,
		NRF24_RX_PW_P5
};

// Addresses of the address registers
static const uint8_t NRF24_ADDR_REGS[7] = {
		NRF24_RX_ADDR_P0,
		NRF24_RX_ADDR_P1,
		NRF24_RX_ADDR_P2,
		NRF24_RX_ADDR_P3,
		NRF24_RX_ADDR_P4,
		NRF24_RX_ADDR_P5,
		NRF24_TX_ADDR
};

typedef enum {
	NRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	NRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	NRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	NRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} NRF24_TXResult;

void NRF24_Init(void);
uint8_t NRF24_Check(void);
void NRF24_RX_Mode(void);
unsigned char NRF24_readWriteReg(unsigned char reg, unsigned char value);
unsigned char NRF24_writeBuf(unsigned char reg, unsigned char *pBuf, unsigned char bytes);
uint8_t NRF24_readReg(unsigned char reg);

uint8_t NRF24_readWrite(uint8_t byte);
void NRF24_WritePayload(uint8_t *pBuf, uint8_t length);
void NRF24_WriteAckPayload(uint8_t *pBuf, uint8_t length);
NRF24_RXResult NRF24_ReadPayload(uint8_t *pBuf, uint8_t *length);
uint8_t NRF24_GetStatus(void);
// void NRF24_ClearIRQFlags(void);
void NRF24_ClearIRQFlags(uint8_t irq_mask);
void NRF24_FlushTX(void);
void NRF24_FlushRX(void);
void NRF24_SetDataRate(uint8_t data_rate);
void NRF24_SetRFChannel(uint8_t channel);
void NRF24_SetAddrWidth(uint8_t addr_width);
void NRF24_SetAddr(uint8_t pipe, const uint8_t *addr);
void NRF24_SetTXPower(uint8_t tx_pwr);
void NRF24_SetAutoRetr(uint8_t ard, uint8_t arc);
void NRF24_EnableAA(uint8_t pipe);
void NRF24_DisableAA(uint8_t pipe);
void NRF24_SetOperationalMode(uint8_t mode);
void NRF24_SetPowerMode(uint8_t mode);
void NRF24_SetCRCScheme(uint8_t scheme);
void NRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len);
void NRF24_SetActivateDPL(void);
void NRF24_SetDPL(void);
void NRF24_SetAckPay(void);
void NRF24_SetDynAsk(void);
void NRF24_SetDplPipe(uint8_t numpipe, uint8_t enable);
void NRF24_ClosePipe(uint8_t pipe);
uint8_t NRF24_GetRetransmitCounters(void);
uint8_t NRF24_GetStatus(void);
uint8_t NRF24_GetIRQFlags(void);
uint8_t NRF24_GetStatus_RXFIFO(void);
uint8_t NRF24_GetStatus_TXFIFO(void);
uint8_t NRF24_GetRXSource(void);
uint8_t NRF24_GetAckPayloadWidth(void);
void NRF24_ResetPLOS(void);
unsigned char NRF24_readBuf(unsigned char reg,unsigned char *pBuf, unsigned char bytes);

