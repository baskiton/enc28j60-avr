/*
    Library for ENC28J60 - Stand-Alone Ethernet Controller with SPI Interface.
    Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/39662e.pdf
    ERRATA: http://ww1.microchip.com/downloads/en/DeviceDoc/80349c.pdf

    Connection example for arduino nano:
        ETHER   NANO
        SCK     D13 (SPI SCK)
        SO      D12 (SPI MISO)
        SI      D11 (SPI MOSI)
        CS      D10 (SPI !SS)
        INT     D2
        RESET   RST
        VCC     3V3
        GND     GND
*/
#ifndef ENC28J60_H
#define ENC28J60_H

#include <stdint.h>

#include <defines.h>

/*! TODO: Move to <defines.h> */
struct avr_pin_s {
    uint8_t pin_num;
    volatile uint8_t *port;
};

/*! TODO: Move to <spi.h> */
struct spi_device_s {
    struct avr_pin_s cs;    // chip select
    struct avr_pin_s rst;   // reset
    struct avr_pin_s intr;  // interrupt
    struct avr_pin_s a0;    // a0
};


#define MAC_ADDR_MAX_LEN 6U
#define ENC28J60_TSV_SIZE 7U
#define ENC28J60_RSV_SIZE 6U
#define ENC28J60_MAX_FREQ 20000000U
#define ENC28J60_MAX_FRAME_LEN 1518U

/* TX buffer pointers to internal 8K RAM.
 * there is enough space for one ethernet frame.
 */
#define ENC28J60_TXSTART_INIT 0x1A00U
#define ENC28J60_TXEND_INIT 0x1FFFU

/* RX buffer pointer to internal 8K RAM.
 * RX buffer must start at 0 (by ERRATA issue #5)
 */
#define ENC28J60_RXSTART_INIT 0x0000U
#define ENC28J60_RXEND_INIT 0x19FFU

/* SPI INSTRUCTION SET FOR THE ENC28J60 */
#define ENC28J60_RCR_OP 0U                      // read control register opcode
#define ENC28J60_RBM_OP ((1U << 5U) | 0b11010U) // read buffer memory opcode
#define ENC28J60_WCR_OP (2U << 5U)              // write control register opcode
#define ENC28J60_WBM_OP ((3U << 5U) | 0b11010U) // write buffer memory opcode
#define ENC28J60_BFS_OP (4U << 5U)              // bit field set opcode
#define ENC28J60_BFC_OP (5U << 5U)              // bit field clear opcode
#define ENC28J60_SRC_OP ((7U << 5U) | 0b11111U) // system reset command opcode (soft reset)

/* 
 * ENC28J60 Control registers
 *      register address    (bits 0-4)
 *      bank number         (bits 5-6)
 *      MAC/MII register    (bit 7)
 */
#define ENC28J60_ADDR_MASK 0x1FU
#define ENC28J60_BANK_MASK 0x60U
#define ENC28J60_DUMMY_MASK 0x80U

/* Common control registers (for all banks) */
#define ENC28J60_EIE    0x1B    // INTIE    PKTIE   DMAIE   LINKIE  TXIE    r       TXERIE  RXERIE
#define ENC28J60_EIR    0x1C    // —        PKTIF   DMAIF   LINKIF  TXIF    r       TXERIF  RXERIF
#define ENC28J60_ESTAT  0x1D    // INT      BUFER   r       LATECOL —       RXBUSY  TXABRT  CLKRDY
#define ENC28J60_ECON2  0x1E    // AUTOINC  PKTDEC  PWRSV   r       VRPS    —       —       —
#define ENC28J60_ECON1  0x1F    // TXRST    RXRST   DMAST   CSUMEN  TXRTS   RXEN    BSEL1   BSEL0

enum EIE_bits {     // 0000_0000 on reset
    RXERIE, // Receive Error Interrupt Enable bit
    TXERIE, // Transmit Error Interrupt Enable bit
    TXIE = 3U,  // Transmit Interrupt Enable bit
    LINKIE, // Link Status Change Interrupt Enable bit
    DMAIE,  // DMA Interrupt Enable bit
    PKTIE,  // Receive Packet Pending Interrupt Enable bit
    INTIE   // Global INT Interrupt Enable bit
};

enum EIR_bits {     // -000_0000 on reset
    RXERIF, // Receive Error Interrupt Flag bit
    TXERIF, // Transmit Error Interrupt Flag bit
    TXIF = 3U,  // Transmit Interrupt Flag bit
    LINKIF, // Link Change Interrupt Flag bit
    DMAIF,  // DMA Interrupt Flag bit
    PKTIF   // Receive Packet Pending Interrupt Flag bit
};

enum ESTAT_bits {   // 0000_-000 on reset
    CLKRDY,
    TXABRT,
    RXBUSY,
    LATECOL = 4U,
    BUFER = 6U,
    INT
};

enum ECON2_bits {   // 1000_0--- on reset
    VRPS = 3U,  // Voltage Regulator Power Save Enable bit
    PWRSV = 5U, // Power Save Enable bit
    PKTDEC,     // Packet Decrement bit
    AUTOINC,    // Automatic Buffer Pointer Increment Enable bit
};

enum ECON1_bits {   // 0000_0000 on reset
    BSEL0,  // Bank Select bit 0
    BSEL1,  // Bank Select bit 1
    RXEN,   // Receive Enable bit
    TXRTS,  // Transmit Request to Send bit
    CSUMEN, // DMA Checksum Enable bit
    DMAST,  // DMA Start and Busy Status bit
    RXRST,  // Receive Logic Reset bit
    TXRST   // Transmit Logic Reset bit
};

/* Bank 0                   DUMMY | BANK | ADDR */
#define ENC28J60_ERDPTL     (0x00 | 0x00 | 0x00)    // Read Pointer Low Byte (ERDPT<7:0>)
#define ENC28J60_ERDPTH     (0x00 | 0x00 | 0x01)    // Read Pointer High Byte (ERDPT<12:8>)
#define ENC28J60_EWRPTL     (0x00 | 0x00 | 0x02)    // Write Pointer Low Byte (EWRPT<7:0>)
#define ENC28J60_EWRPTH     (0x00 | 0x00 | 0x03)    // Write Pointer High Byte (EWRPT<12:8>)
#define ENC28J60_ETXSTL     (0x00 | 0x00 | 0x04)    // TX Start Low Byte (ETXST<7:0>)
#define ENC28J60_ETXSTH     (0x00 | 0x00 | 0x05)    // TX Start High Byte (ETXST<12:8>)
#define ENC28J60_ETXNDL     (0x00 | 0x00 | 0x06)    // TX End Low Byte (ETXND<7:0>)
#define ENC28J60_ETXNDH     (0x00 | 0x00 | 0x07)    // TX End High Byte (ETXND<12:8>)
#define ENC28J60_ERXSTL     (0x00 | 0x00 | 0x08)    // RX Start Low Byte (ERXST<7:0>)
#define ENC28J60_ERXSTH     (0x00 | 0x00 | 0x09)    // RX Start High Byte (ERXST<12:8>)
#define ENC28J60_ERXNDL     (0x00 | 0x00 | 0x0A)    //  RX End Low Byte (ERXND<7:0>)
#define ENC28J60_ERXNDH     (0x00 | 0x00 | 0x0B)    // RX End High Byte (ERXND<12:8>)
#define ENC28J60_ERXRDPTL   (0x00 | 0x00 | 0x0C)    // RX RD Pointer Low Byte (ERXRDPT<7:0>)
#define ENC28J60_ERXRDPTH   (0x00 | 0x00 | 0x0D)    // RX RD Pointer High Byte (ERXRDPT<12:8>)
#define ENC28J60_ERXWRPTL   (0x00 | 0x00 | 0x0E)    // RX WR Pointer Low Byte (ERXWRPT<7:0>)
#define ENC28J60_ERXWRPTH   (0x00 | 0x00 | 0x0F)    // RX WR Pointer High Byte (ERXWRPT<12:8>)
#define ENC28J60_EDMASTL    (0x00 | 0x00 | 0x10)    // DMA Start Low Byte (EDMAST<7:0>)
#define ENC28J60_EDMASTH    (0x00 | 0x00 | 0x11)    // DMA Start High Byte (EDMAST<12:8>)
#define ENC28J60_EDMANDL    (0x00 | 0x00 | 0x12)    // DMA End Low Byte (EDMAND<7:0>)
#define ENC28J60_EDMANDH    (0x00 | 0x00 | 0x13)    // DMA End High Byte (EDMAND<12:8>)
#define ENC28J60_EDMADSTL   (0x00 | 0x00 | 0x14)    // DMA Destination Low Byte (EDMADST<7:0>)
#define ENC28J60_EDMADSTH   (0x00 | 0x00 | 0x15)    // DMA Destination High Byte (EDMADST<12:8>)
#define ENC28J60_EDMACSL    (0x00 | 0x00 | 0x16)    // DMA Checksum Low Byte (EDMACS<7:0>)
#define ENC28J60_EDMACSH    (0x00 | 0x00 | 0x17)    // DMA Checksum High Byte (EDMACS<15:8>)

/* Bank 1                   DUMMY | BANK | ADDR */
#define ENC28J60_EHT0       (0x00 | 0x20 | 0x00)    // Hash Table Byte 0 (EHT<7:0>)
#define ENC28J60_EHT1       (0x00 | 0x20 | 0x01)    // Hash Table Byte 1 (EHT<15:8>)
#define ENC28J60_EHT2       (0x00 | 0x20 | 0x02)    // Hash Table Byte 2 (EHT<23:16>)
#define ENC28J60_EHT3       (0x00 | 0x20 | 0x03)    // Hash Table Byte 3 (EHT<31:24>)
#define ENC28J60_EHT4       (0x00 | 0x20 | 0x04)    // Hash Table Byte 4 (EHT<39:32>)
#define ENC28J60_EHT5       (0x00 | 0x20 | 0x05)    // Hash Table Byte 5 (EHT<47:40>)
#define ENC28J60_EHT6       (0x00 | 0x20 | 0x06)    // Hash Table Byte 6 (EHT<55:48>)
#define ENC28J60_EHT7       (0x00 | 0x20 | 0x07)    // Hash Table Byte 7 (EHT<63:56>)
#define ENC28J60_EPMM0      (0x00 | 0x20 | 0x08)    // Pattern Match Mask Byte 0 (EPMM<7:0>)
#define ENC28J60_EPMM1      (0x00 | 0x20 | 0x09)    // Pattern Match Mask Byte 1 (EPMM<15:8>)
#define ENC28J60_EPMM2      (0x00 | 0x20 | 0x0A)    // Pattern Match Mask Byte 2 (EPMM<23:16>)
#define ENC28J60_EPMM3      (0x00 | 0x20 | 0x0B)    // Pattern Match Mask Byte 3 (EPMM<31:24>)
#define ENC28J60_EPMM4      (0x00 | 0x20 | 0x0C)    // Pattern Match Mask Byte 4 (EPMM<39:32>)
#define ENC28J60_EPMM5      (0x00 | 0x20 | 0x0D)    // Pattern Match Mask Byte 5 (EPMM<47:40>)
#define ENC28J60_EPMM6      (0x00 | 0x20 | 0x0E)    // Pattern Match Mask Byte 6 (EPMM<55:48>)
#define ENC28J60_EPMM7      (0x00 | 0x20 | 0x0F)    // Pattern Match Mask Byte 7 (EPMM<63:56>)
#define ENC28J60_EPMCSL     (0x00 | 0x20 | 0x10)    // Pattern Match Checksum Low Byte (EPMCS<7:0>)
#define ENC28J60_EPMCSH     (0x00 | 0x20 | 0x11)    // Pattern Match Checksum High Byte (EPMCS<15:0>)
#define ENC28J60_EPMOL      (0x00 | 0x20 | 0x14)    // Pattern Match Offset Low Byte (EPMO<7:0>)
#define ENC28J60_EPMOH      (0x00 | 0x20 | 0x15)    // Pattern Match Offset High Byte (EPMO<12:8>)
#define ENC28J60_ERXFCON    (0x00 | 0x20 | 0x18)    // UCEN ANDOR CRCEN PMEN MPEN HTEN MCEN BCEN
#define ENC28J60_EPKTCNT    (0x00 | 0x20 | 0x19)    // Ethernet Packet Count

enum ERXFCON_bits {     // 1010_0001 on reset
    BCEN,   // Broadcast Filter Enable bit
    MCEN,   // Multicast Filter Enable bit
    HTEN,   // Hash Table Filter Enable bit
    MPEN,   // Magic Packet™ Filter Enable bit
    PMEN,   // Pattern Match Filter Enable bit
    CRCEN,  // Post-Filter CRC Check Enable bit
    ANDOR,  // AND/OR Filter Select bit
    UCEN    // Unicast Filter Enable bit
};

/* Bank 2                                  DUMMY | BANK | ADDR */
#define ENC28J60_MACON1     (ENC28J60_DUMMY_MASK | 0x40 | 0x00) // — — — r TXPAUS RXPAUS PASSALL MARXEN
#define ENC28J60_MACON3     (ENC28J60_DUMMY_MASK | 0x40 | 0x02) // PADCFG2 PADCFG1 PADCFG0 TXCRCEN PHDREN HFRMEN FRMLNEN FULDPX
#define ENC28J60_MACON4     (ENC28J60_DUMMY_MASK | 0x40 | 0x03) // — DEFER BPEN NOBKOFF — — r r
#define ENC28J60_MABBIPG    (ENC28J60_DUMMY_MASK | 0x40 | 0x04) // Back-to-Back Inter-Packet Gap (BBIPG<6:0>)
#define ENC28J60_MAIPGL     (ENC28J60_DUMMY_MASK | 0x40 | 0x06) // Non-Back-to-Back Inter-Packet Gap Low Byte (MAIPGL<6:0>)
#define ENC28J60_MAIPGH     (ENC28J60_DUMMY_MASK | 0x40 | 0x07) // Non-Back-to-Back Inter-Packet Gap High Byte (MAIPGH<6:0>)
#define ENC28J60_MACLCON1   (ENC28J60_DUMMY_MASK | 0x40 | 0x08) // Retransmission Maximum (RETMAX<3:0>)
#define ENC28J60_MACLCON2   (ENC28J60_DUMMY_MASK | 0x40 | 0x09) // Collision Window (COLWIN<5:0>)
#define ENC28J60_MAMXFLL    (ENC28J60_DUMMY_MASK | 0x40 | 0x0A) // Maximum Frame Length Low Byte (MAMXFL<7:0>)
#define ENC28J60_MAMXFLH    (ENC28J60_DUMMY_MASK | 0x40 | 0x0B) // Maximum Frame Length High Byte (MAMXFL<15:8>)
#define ENC28J60_MICMD      (ENC28J60_DUMMY_MASK | 0x40 | 0x12) // — — — — — — MIISCAN MIIRD
#define ENC28J60_MIREGADR   (ENC28J60_DUMMY_MASK | 0x40 | 0x14) // MII Register Address (MIREGADR<4:0>)
#define ENC28J60_MIWRL      (ENC28J60_DUMMY_MASK | 0x40 | 0x16) // MII Write Data Low Byte (MIWR<7:0>)
#define ENC28J60_MIWRH      (ENC28J60_DUMMY_MASK | 0x40 | 0x17) // MII Write Data High Byte (MIWR<15:8>)
#define ENC28J60_MIRDL      (ENC28J60_DUMMY_MASK | 0x40 | 0x18) // MII Read Data Low Byte (MIRD<7:0>)
#define ENC28J60_MIRDH      (ENC28J60_DUMMY_MASK | 0x40 | 0x19) // MII Read Data High Byte(MIRD<15:8>)

enum MACON1_bits {      // ---0_0000 on reset
    MARXEN,     // MAC Receive Enable bit
    PASSALL,    // Pass All Received Frames Enable bit
    RXPAUS,     // Pause Control Frame Reception Enable bit
    TXPAUS      // Pause Control Frame Transmission Enable bit
};

enum MACON3_bits {      // 0000_0000 on reset
    FULDPX,     // MAC Full-Duplex Enable bit
    FRMLNEN,    // Frame Length Checking Enable bit
    HFRMEN,     // Huge Frame Enable bit
    PHDREN,     // Proprietary Header Enable bit
    TXCRCEN,    // Transmit CRC Enable bit
    PADCFG0,    // Automatic Pad and CRC Configuration bit 0
    PADCFG1,    // Automatic Pad and CRC Configuration bit 1
    PADCFG2     // Automatic Pad and CRC Configuration bit 2
};

enum MACON4_bits {      // -000_--00 on reset
    NOBKOFF = 4U,   // No Backoff Enable bit (applies to half duplex only)
    BPEN,           // No Backoff During Backpressure Enable bit (applies to half duplex only)
    DEFER           // Defer Transmission Enable bit (applies to half duplex only)
};

enum MICMD_bits {       // ----_--00 on reset
    MIIRD,      // MII Read Enable bit
    MIISCAN     // MII Scan Enable bit
};

/* Bank 3                                  DUMMY | BANK | ADDR */
#define ENC28J60_MAADR5     (ENC28J60_DUMMY_MASK | 0x60 | 0x00) // MAC Address Byte 5 (MAADR<15:8>)
#define ENC28J60_MAADR6     (ENC28J60_DUMMY_MASK | 0x60 | 0x01) // MAC Address Byte 6 (MAADR<7:0>)
#define ENC28J60_MAADR3     (ENC28J60_DUMMY_MASK | 0x60 | 0x02) // MAC Address Byte 3 (MAADR<31:24>), OUI Byte 3
#define ENC28J60_MAADR4     (ENC28J60_DUMMY_MASK | 0x60 | 0x03) // MAC Address Byte 4 (MAADR<23:16>)
#define ENC28J60_MAADR1     (ENC28J60_DUMMY_MASK | 0x60 | 0x04) // MAC Address Byte 1 (MAADR<47:40>), OUI Byte 1
#define ENC28J60_MAADR2     (ENC28J60_DUMMY_MASK | 0x60 | 0x05) // MAC Address Byte 2 (MAADR<39:32>), OUI Byte 2
#define ENC28J60_EBSTSD     (0x00                | 0x60 | 0x06) // Built-in Self-Test Fill Seed (EBSTSD<7:0>)
#define ENC28J60_EBSTCON    (0x00                | 0x60 | 0x07) // PSV2 PSV1 PSV0 PSEL TMSEL1 TMSEL0 TME BISTST
#define ENC28J60_EBSTCSL    (0x00                | 0x60 | 0x08) // Built-in Self-Test Checksum Low Byte (EBSTCS<7:0>)
#define ENC28J60_EBSTCSH    (0x00                | 0x60 | 0x09) // Built-in Self-Test Checksum High Byte (EBSTCS<15:8>)
#define ENC28J60_MISTAT     (ENC28J60_DUMMY_MASK | 0x60 | 0x0A) // — — — — r NVALID SCAN BUSY
#define ENC28J60_EREVID     (0x00                | 0x60 | 0x12) // Ethernet Revision ID (EREVID<4:0>) - READ ONLY REGISTER!!!
#define ENC28J60_ECOCON     (0x00                | 0x60 | 0x15) // — — — — — COCON2 COCON1 COCON0
#define ENC28J60_EFLOCON    (0x00                | 0x60 | 0x17) // — — — — — FULDPXS FCEN1 FCEN0
#define ENC28J60_EPAUSL     (0x00                | 0x60 | 0x18) // Pause Timer Value Low Byte (EPAUS<7:0>)
#define ENC28J60_EPAUSH     (0x00                | 0x60 | 0x19) // Pause Timer Value High Byte (EPAUS<15:8>)

enum EBSTCON_bits {     // 0000_0000 on reset
    BISTST, // Built-in Self-Test Start/Busy bit
    TME,    // Test Mode Enable bit
    TMSEL0, // Test Mode Select bit 0
    TMSEL1, // Test Mode Select bit 1
    PSEL,   // Port Select bit
    PSV0,   // Pattern Shift Value bit 0
    PSV1,   // Pattern Shift Value bit 1
    PSV2    // Pattern Shift Value bit 2
};

enum MISTAT_bits {      // ----_0000 on reset
    BUSY,   // MII Management Busy bit
    SCAN,   // MII Management Scan Operation bit
    NVALID  // MII Management Read Data Not Valid bit
};

enum ECOCON_bits {      // reset to ----_-100 on Power-on and ----_-uuu on all other reset
    COCON0,
    COCON1,
    COCON2
};

enum EFLOCON_bits {     // ----_-000 on reset
    FCEN0,
    FCEN1,
    FULDPXS
};

/* PHY registers */
#define ENC28J60_PHCON1     0x00
#define ENC28J60_PHSTAT1    0x01
#define ENC28J60_PHID1      0x02
#define ENC28J60_PHID2      0x03
#define ENC28J60_PHCON2     0x10
#define ENC28J60_PHSTAT2    0x11
#define ENC28J60_PHIE       0x12
#define ENC28J60_PHIR       0x13
#define ENC28J60_PHLCON     0x14

enum PHCON1_bits {  // 00--_00-q_0---_---- on reset
    PDPXMD = 8U,    // PHY Duplex Mode bit
    PPWRSV = 11U,   // PHY Power-Down bit
    PLOOPBK = 14U,  // PHY Loopback bit
    PRST,   // PHY Software Reset bit
};

enum PHSTAT1_bits { // ---1_1---_----_-00- on reset
    JBSTAT = 1U,    // PHY Latching Jabber Status bit
    LLSTAT, // PHY Latching Link Status bit
    PHDPX = 11U,    // PHY Half-Duplex Capable bit
    PFDPX   // PHY Full-Duplex Capable bit
};

enum PHCON2_bits {  // -000_0000_0000_0000 on reset
    HDLDIS = 8U,    // PHY Half-Duplex Loopback Disable bit
    JABBER = 10,    // Jabber Correction Disable bit
    TXDIS = 13U,    // Twisted-Pair Transmitter Disable bit
    FRCLNK  // PHY Force Linkup bit
};

enum PHSTAT2_bits { // --00_00q-_--0-_---- on reset
    PLRITY = 5U,    // Polarity Status bit
    DPXSTAT = 9U,   // PHY Duplex Status bit
    LSTAT,      // PHY Link Status bit (non-latching)
    COLSTAT,    // PHY Collision Status bit
    RXSTAT,     // PHY Receive Status bit
    TXSTAT      // PHY Transmit Status bit
};

enum PHIE_bits {    // 0000_0000_0000_0000 on reset
    PGEIE = 1U, // PHY Global Interrupt Enable bit
    PLNKIE = 4U // PHY Link Change Interrupt Enable bit
};

enum PHIR_bits {    // xxxx_xxxx_xx00_00x0 on reset
    PGIF = 2U,  // PHY Global Interrupt Flag bit
    PLNKIF = 4U // PHY Link Change Interrupt Flag bit
};

enum PHLCON_bits {  // 0011_0100_0010_001x on reset
    STRCH = 1U, // LED Pulse Stretching Enable bit 
    LFRQ0,  // LED Pulse Stretch Time Configuration bit 0
    LFRQ1,  // LED Pulse Stretch Time Configuration bit 1
    LBCFG0, // LEDB Configuration bit 0
    LBCFG1, // LEDB Configuration bit 1
    LBCFG2, // LEDB Configuration bit 2
    LBCFG3, // LEDB Configuration bit 3
    LACFG0, // LEDA Configuration bit 0
    LACFG1, // LEDA Configuration bit 1
    LACFG2, // LEDA Configuration bit 2
    LACFG3  // LEDA Configuration bit 3
};

/* Receive Status Vector */
enum RSV_bits {
    RSV_LONG_DROP_EVENTS,   //  Indicates a packet over 50,000 bit times occurred or that a packet was dropped since the last receive.
    RSV_CARRIER_EVENT = 2U, // Indicates that at some time since the last receive, a carrier event was detected. The carrier event is not associated with this packet. A carrier event is activity on the receive channel that does not result in a packet receive attempt being made.
    RSV_CRC_ERR = 4U,   // Indicates that frame CRC field value does not match the CRC calculated by the MAC.
    RSV_LEN_CHECK_ERR,  // Indicates that frame length field value in the packet does not match the actual data byte length and specifies a valid length.
    RSV_LEN_OOR,    //  Indicates that frame type/length field was larger than 1500 bytes (type field).
    RSV_RX_OK,  // Indicates that at the packet had a valid CRC and no symbol errors.
    RSV_RX_MULTICAST,   // Indicates packet received had a valid Multicast address.
    RSV_RX_BROADCAST,   // Indicates packet received had a valid Broadcast address.
    RSV_DRIBBLE_NIBBLE, // Indicates that after the end of this packet, an additional 1 to 7 bits were received. The extra bits were thrown away.
    RSV_RX_CTRL_FRAME,  // Current frame was recognized as a control frame for having a valid type/length designating it as a control frame.
    RSV_RX_PAUSE_CTRL_FRAME,    // Current frame was recognized as a control frame containing a valid pause frame opcode and a valid destination address.
    RSV_RX_UNKN_OPC,    // Current frame was recognized as a control frame but it contained an unknown opcode.
    RSV_RX_VLAN_TYPE,   // Current frame was recognized as a VLAN tagged frame.
    RSV_ZERO    // always zero
};

extern void enc28j60_init(uint8_t cs_num, volatile uint8_t *cs_port,
                          uint8_t rst_num, volatile uint8_t *rst_port,
                          uint8_t intr_num, volatile uint8_t * intr_port,
                          bool full_duplex);
extern void enc28j60_soft_reset(void);

extern uint8_t enc28j60_read_rev_id(void);
extern uint16_t enc28j60_read_PHY(uint8_t reg);
extern void enc28j60_get_mac(uint8_t *mac_buf);
extern uint16_t enc28j60_get_rx_free_space(void);
extern void enc28j60_packet_receive(void (*rx_handler)(uint8_t *, uint16_t, uint8_t));
extern bool check_link(void);

#endif  /* !ENC28J60_H */
