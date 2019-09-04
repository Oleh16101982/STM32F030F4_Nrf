#include "nrf24.h"
#include "main.h"
#include "stm32f0xx_spi.h"

uint8_t NRF24_readWrite(uint8_t byte)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(NRF24_SPI, SPI_I2S_FLAG_TXE) == RESET);
	/* Send byte through the SPI1 peripheral */
	SPI_SendData8(NRF24_SPI, byte);
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(NRF24_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	/* Return the byte read from the SPI bus */
	return (uint8_t)SPI_ReceiveData8(NRF24_SPI);
}

unsigned char NRF24_readWriteReg(unsigned char reg, unsigned char value)
{
	unsigned char status;
  NRF24_CSN_LOW();
// select register 
  status = NRF24_readWrite(reg);
// set value
	if ((reg != NRF24_FLUSH_TX) && (reg != NRF24_FLUSH_RX) && \
				(reg != NRF24_REUSE_TX_PL) && (reg != NRF24_NOP)) 
	{
// Send register value
		if (reg != NRF24_R_RX_PL_WID)
		{
			NRF24_readWrite(value);	
		}
		else
		{
			status = NRF24_readWrite(value); // not status return !!! return RX payload width !!!
		}
	}
  NRF24_CSN_HIGH();
  return(status);
}

unsigned char NRF24_readReg(unsigned char reg)
{
  unsigned char reg_val;
	NRF24_CSN_LOW();
	NRF24_readWrite(reg);
	reg_val = NRF24_readWrite(NRF24_READ_REG); // (NRF24_NOP);
	NRF24_CSN_HIGH();
	return(reg_val);
}

unsigned char NRF24_readBuf(unsigned char reg,unsigned char *pBuf, unsigned char bytes)
{
	unsigned char status,i;
	NRF24_CSN_LOW();
// Select register to write to and read status byte
	status = NRF24_readWrite(reg);
	
	for(i=0;i<bytes;i++)
	{
		pBuf[i] = NRF24_readWrite(NRF24_NOP);
	}
	NRF24_CSN_HIGH();
  return(status);
}

unsigned char NRF24_writeBuf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
	unsigned char status,i;
	NRF24_CSN_LOW();
	// Select register to write to and read status byte
	status = NRF24_readWrite(reg);
	for(i=0; i<bytes; i++) // then write all byte in buffer(*pBuf)
	{
		NRF24_readWrite(*pBuf++);
	}
	NRF24_CSN_HIGH();
	return(status);
}

// NRF24_RX_Mode don't use

void NRF24_FlushTX(void) {
	NRF24_readWriteReg(NRF24_FLUSH_TX, NRF24_NOP);
}

void NRF24_FlushRX(void) {
	NRF24_readWriteReg(NRF24_FLUSH_RX, NRF24_NOP);
}
void NRF24_ClearIRQFlags(uint8_t irq_mask) {
	uint8_t reg;

	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg  = NRF24_readReg(NRF24_STATUS);
//	reg |= NRF24_MASK_STATUS_IRQ;
	reg |= irq_mask;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_STATUS, reg);
}
// Set transceiver to it's initial state
// note: RX/TX pipe addresses remains untouched
void NRF24_Init(void) 
{
	uint8_t status;
	// Write to registers their initial values
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_CONFIG			, 0x08);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_AA			, 0x3F);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_RXADDR	, 0x03);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_SETUP_AW		, 0x03);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_SETUP_RETR	, 0x03);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RF_CH			, 0x02);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RF_SETUP		, 0x0E);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_STATUS			, 0x00);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RX_PW_P0		, 0x00);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RX_PW_P1		, 0x00);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RX_PW_P2		, 0x00);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RX_PW_P3		, 0x00);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RX_PW_P4		, 0x00);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RX_PW_P5		, 0x00);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_DYNPD			, 0x00);
	
	status = NRF24_readReg(NRF24_FEATURE);		
	if (!status)
	{		
		NRF24_SetActivateDPL();	
		NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_FEATURE, 0x00);		
	}
	else
	{
		NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_FEATURE, 0x00);		
	}
	
//	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_FEATURE		, 0x00);

	// Clear the FIFO's
	NRF24_FlushRX();
	NRF24_FlushTX();

	// Clear any pending interrupt flags
  NRF24_ClearIRQFlags(NRF24_MASK_STATUS_IRQ_RX | NRF24_MASK_STATUS_IRQ_TX | NRF24_MASK_STATUS_IRQ_RT);

	// Deassert CSN pin (chip release)
	NRF24_CSN_HIGH();
}

// Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t NRF24_Check(void) {
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *)NRF24_TEST_ADDR;

	// Write test TX address and read TX_ADDR register
	NRF24_writeBuf(NRF24_WRITE_REG | NRF24_TX_ADDR, ptr, 5);
	NRF24_readBuf(NRF24_READ_REG | NRF24_TX_ADDR, rxbuf, 5);
	// Compare buffers, return error on first mismatch
	for (i = 0; i < 5; i++) 
	{
		if (rxbuf[i] != *ptr++) return 0;
	}
	return 1;
}

void NRF24_SetPowerMode(uint8_t mode) 
{
	uint8_t reg;

	reg = NRF24_readReg(NRF24_CONFIG);
	if (mode == NRF24_PWR_UP) 
	{
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Stanby-I mode with consumption about 26uA
		reg |= NRF24_CONFIG_PWR_UP;
	} 
	else 
	{
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// into power down mode with consumption about 900nA
		reg &= ~NRF24_CONFIG_PWR_UP;
	}
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_CONFIG, reg);
}

void NRF24_SetOperationalMode(uint8_t mode) 
{
	uint8_t reg;

	// Configure PRIM_RX bit of the CONFIG register
	reg  = NRF24_readReg(NRF24_CONFIG);
	reg &= ~NRF24_CONFIG_PRIM_RX;
	reg |= (mode & NRF24_CONFIG_PRIM_RX);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_CONFIG, reg);
}

void NRF24_SetCRCScheme(uint8_t scheme) 
{
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg  = NRF24_readReg(NRF24_CONFIG);
	reg &= ~NRF24_MASK_CRC;
	reg |= (scheme | NRF24_MASK_CRC);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_CONFIG, reg);
}

void NRF24_SetRFChannel(uint8_t channel) 
{
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RF_CH, channel);
}

void NRF24_SetAutoRetr(uint8_t ard, uint8_t arc) 
{
	// Set auto retransmit settings (SETUP_RETR register)
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_SETUP_RETR, (uint8_t)((ard << 4) | (arc & NRF24_MASK_RETR_ARC)));
}

void NRF24_SetAddrWidth(uint8_t addr_width) 
{
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_SETUP_AW, addr_width - 2);
}

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes

void NRF24_SetAddr(uint8_t pipe, const uint8_t *addr) {
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe) {
		case NRF24_PIPETX:
		case NRF24_PIPE0:
		case NRF24_PIPE1:
// Get address width
			addr_width = NRF24_readReg(NRF24_SETUP_AW) + 1;
// Write address in reverse order (LSByte first)
			addr += addr_width;
			NRF24_CSN_LOW();

			NRF24_readWrite(NRF24_WRITE_REG | NRF24_ADDR_REGS[pipe]);
			do {
				NRF24_readWrite(*addr--);
			} while (addr_width--);
			NRF24_CSN_HIGH();
			break;
		case NRF24_PIPE2:
		case NRF24_PIPE3:
		case NRF24_PIPE4:
		case NRF24_PIPE5:
// Write address LSBbyte (only first byte from the addr buffer)
			NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_ADDR_REGS[pipe], *addr);
			break;
		default:
// Incorrect pipe number -> do nothing
			break;
	}
}

void NRF24_SetTXPower(uint8_t tx_pwr) {
	uint8_t reg;

	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg  = NRF24_readReg(NRF24_RF_SETUP);
	reg &= ~NRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RF_SETUP, reg);
}


void NRF24_SetActivateDPL(void)
{
	NRF24_readWriteReg(NRF24_ACTIVATE, 0x73);
}

void NRF24_SetDataRate(uint8_t data_rate) {
	uint8_t reg;

	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
	reg  = NRF24_readReg(NRF24_RF_SETUP);
	reg &= ~NRF24_MASK_DATARATE;
	reg |= data_rate;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RF_SETUP, reg);
}


// set or reset bit Enables Dynamic Payload Length
void NRF24_SetDPL(void)
{
	uint8_t reg;
	reg  = NRF24_readReg(NRF24_FEATURE);
	reg &= ~NRF24_MASK_EN_DPL;
	reg |= NRF24_MASK_EN_DPL;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_FEATURE, reg);		
}
// set or reset bit Enables Payload with ACK
void NRF24_SetAckPay(void)
{
	uint8_t reg;
	reg  = NRF24_readReg(NRF24_FEATURE);
	reg &= ~NRF24_MASK_EN_ACK_PAY;
	reg |= NRF24_MASK_EN_ACK_PAY;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_FEATURE, reg);	
}

void NRF24_SetDynAsk(void)
{
	uint8_t reg;
	reg  = NRF24_readReg(NRF24_FEATURE);
	reg &= ~NRF24_MASK_EN_DYN_ACK;
	reg |= NRF24_MASK_EN_DYN_ACK;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_FEATURE, reg);			
}

void NRF24_SetDplPipe(uint8_t numpipe, uint8_t enable)
{
	uint8_t reg;
	enable <<= numpipe;
	reg  = NRF24_readReg(NRF24_DYNPD);
	reg &= ~enable;
	reg |= enable;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_DYNPD, reg);		
}


// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void NRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len) {
	uint8_t reg;

	// Enable the specified pipe (EN_RXADDR register)
	reg = (NRF24_readReg(NRF24_EN_RXADDR) | (1 << pipe)) & NRF24_MASK_EN_RX;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RX_PW_PIPE[pipe], payload_len & NRF24_MASK_RX_PW);

	// Set auto acknowledgment for a specified pipe (EN_AA register)

	reg = NRF24_readReg(NRF24_EN_AA);	
	if (aa_state == NRF24_AA_ON) {
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_AA, reg);
}

// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
void NRF24_ClosePipe(uint8_t pipe) 
{
	uint8_t reg;

	reg  = NRF24_readReg(NRF24_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= NRF24_MASK_EN_RX;
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_RXADDR, reg);
}

void NRF24_EnableAA(uint8_t pipe) 
{
	uint8_t reg;

	// Set bit in EN_AA register
	reg  = NRF24_readReg(NRF24_EN_AA);
	reg |= (1 << pipe);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_AA, reg);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
void NRF24_DisableAA(uint8_t pipe) 
{
	uint8_t reg;

	if (pipe > 5) {
		// Disable Auto-ACK for ALL pipes
		NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		reg  = NRF24_readReg(NRF24_EN_AA);
		reg &= ~(1 << pipe);
		NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_EN_AA, reg);
	}
}

// Get value of the STATUS register
// return: value of STATUS register
uint8_t NRF24_GetStatus(void) 
{
	return NRF24_readReg(NRF24_STATUS);
}

// Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
uint8_t NRF24_GetIRQFlags(void) 
{
	return (NRF24_readReg(NRF24_STATUS) & NRF24_MASK_STATUS_IRQ);
}

// Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
uint8_t NRF24_GetStatus_RXFIFO(void) 
{
	return (NRF24_readReg(NRF24_FIFO_STATUS) & NRF24_MASK_RXFIFO);
}

// Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
uint8_t NRF24_GetStatus_TXFIFO(void) 
{
	return ((NRF24_readReg(NRF24_FIFO_STATUS) & NRF24_MASK_TXFIFO) >> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t NRF24_GetRXSource(void) 
{
	return ((NRF24_readReg(NRF24_STATUS) & NRF24_MASK_RX_P_NO) >> 1);
}
// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
uint8_t NRF24_GetRetransmitCounters(void) 
{
	return (NRF24_readReg(NRF24_OBSERVE_TX));
}
// read RX payload width for ACK with Payload mode
uint8_t NRF24_GetAckPayloadWidth(void)
{	
	return NRF24_readWriteReg(NRF24_R_RX_PL_WID, NRF24_NOP);
}

// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
void NRF24_ResetPLOS(void) 
{
	uint8_t reg;

	// The PLOS counter is reset after write to RF_CH register
	reg = NRF24_readReg(NRF24_RF_CH);
	NRF24_readWriteReg(NRF24_WRITE_REG | NRF24_RF_CH, reg);
}

void NRF24_WritePayload(uint8_t *pBuf, uint8_t length) 
{
	NRF24_writeBuf(NRF24_WR_TX_PLOAD, pBuf, length);
}

void NRF24_WriteAckPayload(uint8_t *pBuf, uint8_t length) 
{
	NRF24_writeBuf(NRF24_W_ACK_PAYLOAD | 0x01, pBuf, length);
//	NRF24_writeBuf(NRF24_W_ACK_PAYLOAD, pBuf, length);	
}


// Read top level payload available in the RX FIFO
// input:
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
NRF24_RXResult NRF24_ReadPayload(uint8_t *pBuf, uint8_t *length) 
{
	uint8_t pipe;

	// Extract a payload pipe number from the STATUS register
	pipe = (NRF24_readReg(NRF24_STATUS) & NRF24_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {
		// Get payload length
		*length = NRF24_readReg(NRF24_RX_PW_PIPE[pipe]);

		// Read a payload from the RX FIFO
		if (*length) {
			NRF24_readBuf(NRF24_RD_RX_PLOAD, pBuf, *length);
		}

		return ((NRF24_RXResult)pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return NRF24_RX_EMPTY;
}

