/**
 * @file       Enc28j60.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Enc28j60.h"

/*================================ define ===================================*/

// ENC28J60 Control Registers
// Control register definitions are a combination of address,
// bank number, and Ethernet/MAC/PHY indicator bits.
// - Register address       (bits 0-4)
// - Bank number            (bits 5-6)
// - MAC/PHY indicator      (bit 7)
#define ADDR_MASK                   ( 0x1F )
#define BANK_MASK                   ( 0x60 )
#define SPRD_MASK                   ( 0x80 )

// All-bank registers
#define EIE                         ( 0x1B )
#define EIR                         ( 0x1C )
#define ESTAT                       ( 0x1D )
#define ECON2                       ( 0x1E )
#define ECON1                       ( 0x1F )

// Bank 0 registers
#define ERDPT                       ( 0x00 | 0x00 )
#define EWRPT                       ( 0x02 | 0x00 )
#define ETXST                       ( 0x04 | 0x00 )
#define ETXND                       ( 0x06 | 0x00 )
#define ERXST                       ( 0x08 | 0x00 )
#define ERXND                       ( 0x0A | 0x00 )
#define ERXRDPT                     ( 0x0C | 0x00 )
#define ERXWRPT                     ( 0x0E | 0x00 )
#define EDMAST                      ( 0x10 | 0x00 )
#define EDMAND                      ( 0x12 | 0x00 )
#define EDMADST                     ( 0x14 | 0x00 )
#define EDMACS                      ( 0x16 | 0x00 )

// Bank 1 registers
#define EHT0                        ( 0x00 | 0x20 )
#define EHT1                        ( 0x01 | 0x20 )
#define EHT2                        ( 0x02 | 0x20 )
#define EHT3                        ( 0x03 | 0x20 )
#define EHT4                        ( 0x04 | 0x20 )
#define EHT5                        ( 0x05 | 0x20 )
#define EHT6                        ( 0x06 | 0x20 )
#define EHT7                        ( 0x07 | 0x20 )
#define EPMM0                       ( 0x08 | 0x20 )
#define EPMM1                       ( 0x09 | 0x20 )
#define EPMM2                       ( 0x0A | 0x20 )
#define EPMM3                       ( 0x0B | 0x20 )
#define EPMM4                       ( 0x0C | 0x20 )
#define EPMM5                       ( 0x0D | 0x20 )
#define EPMM6                       ( 0x0E | 0x20 )
#define EPMM7                       ( 0x0F | 0x20 )
#define EPMCS                       ( 0x10 | 0x20 )
#define EPMO                        ( 0x14 | 0x20 )
#define EWOLIE                      ( 0x16 | 0x20 )
#define EWOLIR                      ( 0x17 | 0x20 )
#define ERXFCON                     ( 0x18 | 0x20 )
#define EPKTCNT                     ( 0x19 | 0x20 )

// Bank 2 registers
#define MACON1                      ( 0x00 | 0x40 | 0x80 )
#define MACON2                      ( 0x01 | 0x40 | 0x80 )
#define MACON3                      ( 0x02 | 0x40 | 0x80 )
#define MACON4                      ( 0x03 | 0x40 | 0x80 )
#define MABBIPG                     ( 0x04 | 0x40 | 0x80 )
#define MAIPG                       ( 0x06 | 0x40 | 0x80 )
#define MACLCON1                    ( 0x08 | 0x40 | 0x80 )
#define MACLCON2                    ( 0x09 | 0x40 | 0x80 )
#define MAMXFL                      ( 0x0A | 0x40 | 0x80 )
#define MAPHSUP                     ( 0x0D | 0x40 | 0x80 )
#define MICON                       ( 0x11 | 0x40 | 0x80 )
#define MICMD                       ( 0x12 | 0x40 | 0x80 )
#define MIREGADR                    ( 0x14 | 0x40 | 0x80 )
#define MIWR                        ( 0x16 | 0x40 | 0x80 )
#define MIRD                        ( 0x18 | 0x40 | 0x80 )
// Bank 3 registers
#define MAADR1                      ( 0x00 | 0x60 | 0x80 )
#define MAADR0                      ( 0x01 | 0x60 | 0x80 )
#define MAADR3                      ( 0x02 | 0x60 | 0x80 )
#define MAADR2                      ( 0x03 | 0x60 | 0x80 )
#define MAADR5                      ( 0x04 | 0x60 | 0x80 )
#define MAADR4                      ( 0x05 | 0x60 | 0x80 )
#define EBSTSD                      ( 0x06 | 0x60 )
#define EBSTCON                     ( 0x07 | 0x60 )
#define EBSTCS                      ( 0x08 | 0x60 )
#define MISTAT                      ( 0x0A | 0x60 | 0x80 )
#define EREVID                      ( 0x12 | 0x60 )
#define ECOCON                      ( 0x15 | 0x60 )
#define EFLOCON                     ( 0x17 | 0x60 )
#define EPAUS                       ( 0x18 | 0x60 )

// ENC28J60 ERXFCON Register Bit Definitions
#define ERXFCON_UCEN                ( 0x80 )
#define ERXFCON_ANDOR               ( 0x40 )
#define ERXFCON_CRCEN               ( 0x20 )
#define ERXFCON_PMEN                ( 0x10 )
#define ERXFCON_MPEN                ( 0x08 )
#define ERXFCON_HTEN                ( 0x04 )
#define ERXFCON_MCEN                ( 0x02 )
#define ERXFCON_BCEN                ( 0x01 )

// ENC28J60 EIE Register Bit Definitions
#define EIE_INTIE                   ( 0x80 )
#define EIE_PKTIE                   ( 0x40 )
#define EIE_DMAIE                   ( 0x20 )
#define EIE_LINKIE                  ( 0x10 )
#define EIE_TXIE                    ( 0x08 )
#define EIE_WOLIE                   ( 0x04 )
#define EIE_TXERIE                  ( 0x02 )
#define EIE_RXERIE                  ( 0x01 )

// ENC28J60 EIR Register Bit Definitions
#define EIR_PKTIF                   ( 0x40 )
#define EIR_DMAIF                   ( 0x20 )
#define EIR_LINKIF                  ( 0x10 )
#define EIR_TXIF                    ( 0x08 )
#define EIR_WOLIF                   ( 0x04 )
#define EIR_TXERIF                  ( 0x02 )
#define EIR_RXERIF                  ( 0x01 )

// ENC28J60 ESTAT Register Bit Definitions
#define ESTAT_INT                   ( 0x80 )
#define ESTAT_LATECOL               ( 0x10 )
#define ESTAT_RXBUSY                ( 0x04 )
#define ESTAT_TXABRT                ( 0x02 )
#define ESTAT_CLKRDY                ( 0x01 )

// ENC28J60 ECON2 Register Bit Definitions
#define ECON2_AUTOINC               ( 0x80 )
#define ECON2_PKTDEC                ( 0x40 )
#define ECON2_PWRSV                 ( 0x20 )
#define ECON2_VRPS                  ( 0x08 )

// ENC28J60 ECON1 Register Bit Definitions
#define ECON1_TXRST                 ( 0x80 )
#define ECON1_RXRST                 ( 0x40 )
#define ECON1_DMAST                 ( 0x20 )
#define ECON1_CSUMEN                ( 0x10 )
#define ECON1_TXRTS                 ( 0x08 )
#define ECON1_RXEN                  ( 0x04 )
#define ECON1_BSEL1                 ( 0x02 )
#define ECON1_BSEL0                 ( 0x01 )

// ENC28J60 MACON1 Register Bit Definitions
#define MACON1_LOOPBK               ( 0x10 )
#define MACON1_TXPAUS               ( 0x08 )
#define MACON1_RXPAUS               ( 0x04 )
#define MACON1_PASSALL              ( 0x02 )
#define MACON1_MARXEN               ( 0x01 )

// ENC28J60 MACON2 Register Bit Definitions
#define MACON2_MARST                ( 0x80 )
#define MACON2_RNDRST               ( 0x40 )
#define MACON2_MARXRST              ( 0x08 )
#define MACON2_RFUNRST              ( 0x04 )
#define MACON2_MATXRST              ( 0x02 )
#define MACON2_TFUNRST              ( 0x01 )

// ENC28J60 MACON3 Register Bit Definitions
#define MACON3_PADCFG2              ( 0x80 )
#define MACON3_PADCFG1              ( 0x40 )
#define MACON3_PADCFG0              ( 0x20 )
#define MACON3_TXCRCEN              ( 0x10 )
#define MACON3_PHDRLEN              ( 0x08 )
#define MACON3_HFRMLEN              ( 0x04 )
#define MACON3_FRMLNEN              ( 0x02 )
#define MACON3_FULDPX               ( 0x01 )

// ENC28J60 MICMD Register Bit Definitions
#define MICMD_MIISCAN               ( 0x02 )
#define MICMD_MIIRD                 ( 0x01 )

// ENC28J60 MISTAT Register Bit Definitions
#define MISTAT_NVALID               ( 0x04 )
#define MISTAT_SCAN                 ( 0x02 )
#define MISTAT_BUSY                 ( 0x01 )

// ENC28J60 EBSTCON Register Bit Definitions
#define EBSTCON_PSV2                ( 0x80 )
#define EBSTCON_PSV1                ( 0x40 )
#define EBSTCON_PSV0                ( 0x20 )
#define EBSTCON_PSEL                ( 0x10 )
#define EBSTCON_TMSEL1              ( 0x08 )
#define EBSTCON_TMSEL0              ( 0x04 )
#define EBSTCON_TME                 ( 0x02 )
#define EBSTCON_BISTST              ( 0x01 )

// PHY registers
#define PHCON1                      ( 0x00 )
#define PHSTAT1                     ( 0x01 )
#define PHHID1                      ( 0x02 )
#define PHHID2                      ( 0x03 )
#define PHCON2                      ( 0x10 )
#define PHSTAT2                     ( 0x11 )
#define PHIE                        ( 0x12 )
#define PHIR                        ( 0x13 )
#define PHLCON                      ( 0x14 )

// ENC28J60 PHY PHCON1 Register Bit Definitions
#define PHCON1_PRST                 ( 0x8000 )
#define PHCON1_PLOOPBK              ( 0x4000 )
#define PHCON1_PPWRSV               ( 0x0800 )
#define PHCON1_PDPXMD               ( 0x0100 )

// ENC28J60 PHY PHSTAT1 Register Bit Definitions
#define PHSTAT1_PFDPX               ( 0x1000 )
#define PHSTAT1_PHDPX               ( 0x0800 )
#define PHSTAT1_LLSTAT              ( 0x0004 )
#define PHSTAT1_JBSTAT              ( 0x0002 )

// ENC28J60 PHY PHCON2 Register Bit Definitions
#define PHCON2_FRCLINK              ( 0x4000 )
#define PHCON2_TXDIS                ( 0x2000 )
#define PHCON2_JABBER               ( 0x0400 )
#define PHCON2_HDLDIS               ( 0x0100 )

// ENC28J60 Packet Control Byte Bit Definitions
#define PKTCTRL_PHUGEEN             ( 0x08 )
#define PKTCTRL_PPADEN              ( 0x04 )
#define PKTCTRL_PCRCEN              ( 0x02 )
#define PKTCTRL_POVERRIDE           ( 0x01 )

// SPI operation codes
#define ENC28J60_READ_CTRL_REG      ( 0x00 )
#define ENC28J60_READ_BUF_MEM       ( 0x3A )
#define ENC28J60_WRITE_CTRL_REG     ( 0x40 )
#define ENC28J60_WRITE_BUF_MEM      ( 0x7A )
#define ENC28J60_BIT_FIELD_SET      ( 0x80 )
#define ENC28J60_BIT_FIELD_CLR      ( 0xA0 )
#define ENC28J60_SOFT_RESET         ( 0xFF )

// According to Errata #5, RXSTART_INIT must be zero
#define RXSTART_INIT                ( 0x0000 )  // Start of RX buffer
#define RXSTOP_INIT                 ( 0x0BFF )  // End of RX buffer

#define TXSTART_INIT                ( 0x0C00 )  // Start of TX buffer
#define TXSTOP_INIT                 ( 0x11FF )  // End of TX buffer

// Maximum frame length which the controller will accept
#define MAX_FRAMELEN                ( 1518 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Enc28j60::Enc28j60(Spi& spi, GpioIn& gpio):
    spi_(spi), gpio_(gpio), \
    interrupt_(this, &Enc28j60::interruptHandler), \
    callback_(nullptr), \
    nextPacketPtr(0)
{
}

void Enc28j60::init(uint8_t* mac_address)
{
    // Set the MAC address
    setMacAddress(mac_address);

    // Initialize the ENC28J60 chip
    reset();

    // Set and enable ENC268J60 interrupt
    gpio_.setCallback(&interrupt_);
    gpio_.enableInterrupts();
}

void Enc28j60::reset(void)
{
    // Trigger a software reset
    writeOperation(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);

    // Errata #2: Wait for at least 1 ms after software reset
    for (uint32_t i = 0x1FFF; i != 0; i--)
        ;

    // Wait until the clock becomes ready
    while(!readOperation(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY);

    // Store a pointer to the next packet
    nextPacketPtr = RXSTART_INIT;

    // Set the packet transmit and receive buffers
    writeRegister(ERXST, RXSTART_INIT);
    writeRegister(ERXRDPT, RXSTART_INIT);
    writeRegister(ERXND, RXSTOP_INIT);
    writeRegister(ETXST, TXSTART_INIT);
    writeRegister(ETXND, TXSTOP_INIT);

    // Set MACON1 register
    writeRegisterByte(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);

    // Set MACON2 register
    writeRegisterByte(MACON2, 0x00);

    // Set MACON3 registers
    writeOperation(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);

    // Set Non-Back-to-Back Inter-Packet Gap
    writeRegister(MAIPG, 0x0C12);

    // Set Back-to-Back Inter-Packet Gap
    writeRegisterByte(MABBIPG, 0x12);

    // Set maximum frame length
    writeRegister(MAMXFL, MAX_FRAMELEN);

    // Set MAC address
    writeRegisterByte(MAADR5, macAddress[0]);
    writeRegisterByte(MAADR4, macAddress[1]);
    writeRegisterByte(MAADR3, macAddress[2]);
    writeRegisterByte(MAADR2, macAddress[3]);
    writeRegisterByte(MAADR1, macAddress[4]);
    writeRegisterByte(MAADR0, macAddress[5]);

    // Errata #9/10: Disable loopback in half-duplex
    writePhy(PHCON2, PHCON2_HDLDIS);

    // Set ECON1 bank
    setBank(ECON1);

    // Enable packet interrupt
    writeOperation(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);

    // Enable packet reception
    writeOperation(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

void Enc28j60::setCallback(Callback* callback)
{
    callback_ = callback;
}

void Enc28j60::clearCallback(void)
{
    callback_ = nullptr;
}

OperationResult Enc28j60::transmitFrame(uint8_t* data, uint32_t length)
{
    // Errata #12: In half-duplex, transmit logic may stall
    while (readOperation(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS)
    {
        if (readRegisterByte(EIR) & EIR_TXERIF)
        {
            writeOperation(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
            writeOperation(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
        }
    }

    // Set transmit buffer start and end
    writeRegister(EWRPT, TXSTART_INIT);
    writeRegister(ETXND, TXSTART_INIT + length);

    // Use default per packet control bytes
    writeOperation(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

    // Write data to buffer
    writeBuffer(data, length);

    // Enable transmission
    writeOperation(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);

    return ResultSuccess;
}

OperationResult Enc28j60::receiveFrame(uint8_t* buffer, uint32_t* length)
{
    uint32_t payloadLength = 0;

    if (readRegisterByte(EPKTCNT) > 0)
    {
        writeRegister(ERDPT, nextPacketPtr);

        struct {
            uint16_t nextPacket;
            uint16_t byteCount;
            uint16_t status;
        } receiveHeader;

        // Read the header
        readBuffer((uint8_t*) &receiveHeader, sizeof(receiveHeader));

        // Update the pointer to the next packet
        nextPacketPtr = receiveHeader.nextPacket;

         // Remove the CRC count
        payloadLength = receiveHeader.byteCount - 4;

        // Check for buffer overflow
        if (payloadLength > *length - 1)
        {
            payloadLength = *length - 1;
        }

        // Check for CRC errors
        if ((receiveHeader.status & 0x0080) == 0)
        {
            return ResultError;
        }
        else
        {
            // Copy the packet to the buffer
            readBuffer(buffer, payloadLength);
        }

        // Clear last buffer position
        buffer[payloadLength] = 0;

        // Errata #14: Receive hardware may corrupt receive buffer
        if (nextPacketPtr - 1 > RXSTOP_INIT)
        {
            writeRegister(ERXRDPT, RXSTOP_INIT);
        }
        else
        {
            writeRegister(ERXRDPT, nextPacketPtr - 1);
        }

        // Decrement the number of packets
        writeOperation(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
    }

    return ResultSuccess;
}

/*=============================== protected =================================*/

void Enc28j60::interruptHandler(void)
{
    if (callback_ != nullptr)
    {
        callback_->execute();
    }
}

/*================================ private ==================================*/

uint8_t Enc28j60::readOperation(uint8_t op, uint8_t address)
{
    uint8_t result;

    spi_.select();

    spi_.writeByte(op | (address & ADDR_MASK));

    result = spi_.readByte();
    if (address & 0x80)
    {
        result = spi_.readByte();
    }

    spi_.deselect();

    return result;
}

void Enc28j60::writeOperation(uint8_t op, uint8_t address, uint8_t data)
{
    spi_.select();

    spi_.writeByte(op | (address & ADDR_MASK));
    spi_.writeByte(data);

    spi_.deselect();
}

void Enc28j60::setBank(uint8_t address)
{
    if ((address & BANK_MASK) != currentBank)
    {
        writeOperation(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
        currentBank = address & BANK_MASK;
        writeOperation(ENC28J60_BIT_FIELD_SET, ECON1, currentBank >> 5);
    }
}

uint8_t Enc28j60::readRegisterByte (uint8_t address)
{
    setBank(address);
    return readOperation(ENC28J60_READ_CTRL_REG, address);
}

void Enc28j60::writeRegisterByte(uint8_t address, uint8_t data)
{
    setBank(address);
    writeOperation(ENC28J60_WRITE_CTRL_REG, address, data);
}

uint16_t Enc28j60::readRegister(uint8_t address)
{
    return readRegisterByte(address) + (readRegisterByte(address + 1) << 8);
}

void Enc28j60::writeRegister(uint8_t address, uint16_t data)
{
    writeRegisterByte(address, data);
    writeRegisterByte(address + 1, data >> 8);
}

uint16_t Enc28j60::readPhyByte(uint8_t address)
{
    writeRegisterByte(MIREGADR, address);
    writeRegisterByte(MICMD, MICMD_MIIRD);

    while (readRegisterByte(MISTAT) & MISTAT_BUSY);

    writeRegisterByte(MICMD, 0x00);

    return readRegisterByte(MIRD+1);
}

void Enc28j60::writePhy(uint8_t address, uint16_t data)
{
    writeRegisterByte(MIREGADR, address);

    writeRegister(MIWR, data);

    while (readRegisterByte(MISTAT) & MISTAT_BUSY)
        ;
}
void Enc28j60::writeBuffer(const uint8_t* data, uint16_t length)
{
    spi_.select();

    spi_.writeByte(ENC28J60_WRITE_BUF_MEM);
    while (length--)
    {
        spi_.writeByte(*data++);
    }

    spi_.deselect();
}

void Enc28j60::readBuffer(uint8_t* data, uint16_t length)
{
    spi_.select();

    spi_.writeByte(ENC28J60_READ_BUF_MEM);
    while (length--)
    {
        *data++ = spi_.readByte();
    }

    spi_.deselect();
}

bool Enc28j60::isLinkUp(void)
{
    return (readPhyByte(PHSTAT2) >> 2) & 0x01;
}
