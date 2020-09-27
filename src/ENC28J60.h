/*
    Library for ENC28J60 - Stand-Alone Ethernet Controller with SPI Interface.
    Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/39662e.pdf

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
struct avr_pin {
    uint8_t pin_num;
    volatile uint8_t *port;
};

typedef struct eth_controls {
    struct avr_pin cs;
    struct avr_pin rst;
    struct avr_pin intr;
} eth_pins;

union mac_u {
  uint64_t mac_64;
  struct {
    uint8_t mac_6;
    uint8_t mac_5;
    uint8_t mac_4;
    uint8_t mac_3;
    uint8_t mac_2;
    uint8_t mac_1;
  } mac_s;
};

#define ETHER_FREQ 20000000U
#define ETHER_MAX_FRAME_LEN 1518U

/* Control registers */
enum CR_COMMON {    // Common control registers (for all banks)
    ENC28J60_EIE = 0x1BU,   // INTIE    PKTIE   DMAIE   LINKIE  TXIE    r       TXERIE  RXERIE
    ENC28J60_EIR,           // —        PKTIF   DMAIF   LINKIF  TXIF    r       TXERIF  RXERIF
    ENC28J60_ESTAT,         // INT      BUFER   r       LATECOL —       RXBUSY  TXABRT  CLKRDY
    ENC28J60_ECON2,         // AUTOINC  PKTDEC  PWRSV   r       VRPS    —       —       —
    ENC28J60_ECON1          // TXRST    RXRST   DMAST   CSUMEN  TXRTS   RXEN    BSEL1   BSEL0
};

enum CR_BANK_0 {    // for bank 0
    ENC28J60_ERDPTL,    // Read Pointer Low Byte (ERDPT<7:0>)
    ENC28J60_ERDPTH,    // Read Pointer High Byte (ERDPT<12:8>)
    ENC28J60_EWRPTL,    // Write Pointer Low Byte (EWRPT<7:0>)
    ENC28J60_EWRPTH,    // Write Pointer High Byte (EWRPT<12:8>)
    ENC28J60_ETXSTL,    // TX Start Low Byte (ETXST<7:0>)
    ENC28J60_ETXSTH,    // TX Start High Byte (ETXST<12:8>)
    ENC28J60_ETXNDL,    // TX End Low Byte (ETXND<7:0>)
    ENC28J60_ETXNDH,    // TX End High Byte (ETXND<12:8>)
    ENC28J60_ERXSTL,    // RX Start Low Byte (ERXST<7:0>)
    ENC28J60_ERXSTH,    // RX Start High Byte (ERXST<12:8>)
    ENC28J60_ERXNDL,    //  RX End Low Byte (ERXND<7:0>)
    ENC28J60_ERXNDH,    // RX End High Byte (ERXND<12:8>)
    ENC28J60_ERXRDPTL,  // RX RD Pointer Low Byte (ERXRDPT<7:0>)
    ENC28J60_ERXRDPTH,  // RX RD Pointer High Byte (ERXRDPT<12:8>)
    ENC28J60_ERXWRPTL,  // RX WR Pointer Low Byte (ERXWRPT<7:0>)
    ENC28J60_ERXWRPTH,  // RX WR Pointer High Byte (ERXWRPT<12:8>)
    ENC28J60_EDMASTL,   // DMA Start Low Byte (EDMAST<7:0>)
    ENC28J60_EDMASTH,   // DMA Start High Byte (EDMAST<12:8>)
    ENC28J60_EDMANDL,   // DMA End Low Byte (EDMAND<7:0>)
    ENC28J60_EDMANDH,   // DMA End High Byte (EDMAND<12:8>)
    ENC28J60_EDMADSTL,  // DMA Destination Low Byte (EDMADST<7:0>)
    ENC28J60_EDMADSTH,  // DMA Destination High Byte (EDMADST<12:8>)
    ENC28J60_EDMACSL,   // DMA Checksum Low Byte (EDMACS<7:0>)
    ENC28J60_EDMACSH    // DMA Checksum High Byte (EDMACS<15:8>)
};

enum EIE_bits {     // 0000_0000 on reset
    RXERIE,
    TXERIE,
    TXIE = 3U,
    LINKIE,
    DMAIE,
    PKTIE,
    INTIE
};

enum EIR_bits {     // -000_0000 on reset
    RXERIF,
    TXERIF,
    TXIF = 3U,
    LINKIF,
    DMAIF,
    PKTIF
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

enum CR_BANK_1 {    // for bank 1
    ENC28J60_EHT0,      // Hash Table Byte 0 (EHT<7:0>)
    ENC28J60_EHT1,      // Hash Table Byte 1 (EHT<15:8>)
    ENC28J60_EHT2,      // Hash Table Byte 2 (EHT<23:16>)
    ENC28J60_EHT3,      // Hash Table Byte 3 (EHT<31:24>)
    ENC28J60_EHT4,      // Hash Table Byte 4 (EHT<39:32>)
    ENC28J60_EHT5,      // Hash Table Byte 5 (EHT<47:40>)
    ENC28J60_EHT6,      // Hash Table Byte 6 (EHT<55:48>)
    ENC28J60_EHT7,      // Hash Table Byte 7 (EHT<63:56>)
    ENC28J60_EPMM0,     // Pattern Match Mask Byte 0 (EPMM<7:0>)
    ENC28J60_EPMM1,     // Pattern Match Mask Byte 1 (EPMM<15:8>)
    ENC28J60_EPMM2,     // Pattern Match Mask Byte 2 (EPMM<23:16>)
    ENC28J60_EPMM3,     // Pattern Match Mask Byte 3 (EPMM<31:24>)
    ENC28J60_EPMM4,     // Pattern Match Mask Byte 4 (EPMM<39:32>)
    ENC28J60_EPMM5,     // Pattern Match Mask Byte 5 (EPMM<47:40>)
    ENC28J60_EPMM6,     // Pattern Match Mask Byte 6 (EPMM<55:48>)
    ENC28J60_EPMM7,     // Pattern Match Mask Byte 7 (EPMM<63:56>)
    ENC28J60_EPMCSL,    // Pattern Match Checksum Low Byte (EPMCS<7:0>)
    ENC28J60_EPMCSH,    // Pattern Match Checksum High Byte (EPMCS<15:0>)
    ENC28J60_EPMOL = 0x14U,     // Pattern Match Offset Low Byte (EPMO<7:0>)
    ENC28J60_EPMOH,     //  Pattern Match Offset High Byte (EPMO<12:8>)
    ENC28J60_ERXFCON = 0x18U,   // UCEN     ANDOR   CRCEN   PMEN    MPEN    HTEN    MCEN    BCEN
    ENC28J60_EPKTCNT    // Ethernet Packet Count
};

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

enum CR_BANK_2 {    // for bank 2
    ENC28J60_MACON1,    // —        —       —       r       TXPAUS  RXPAUS  PASSALL MARXEN
    ENC28J60_MACON3 = 0x02U,    // PADCFG2  PADCFG1 PADCFG0 TXCRCEN PHDREN  HFRMEN  FRMLNEN FULDPX 
    ENC28J60_MACON4,    // —        DEFER   BPEN    NOBKOFF —       —       r       r 
    ENC28J60_MABBIPG,   // Back-to-Back Inter-Packet Gap (BBIPG<6:0>)
    ENC28J60_MAIPGL = 0x06U,    // Non-Back-to-Back Inter-Packet Gap Low Byte (MAIPGL<6:0>)
    ENC28J60_MAIPGH,            // Non-Back-to-Back Inter-Packet Gap High Byte (MAIPGH<6:0>)
    ENC28J60_MACLCON1,  // Retransmission Maximum (RETMAX<3:0>)
    ENC28J60_MACLCON2,  // Collision Window (COLWIN<5:0>)
    ENC28J60_MAMXFLL,   // Maximum Frame Length Low Byte (MAMXFL<7:0>)
    ENC28J60_MAMXFLH,   //  Maximum Frame Length High Byte (MAMXFL<15:8>)
    ENC28J60_MICMD = 0x12U,     // — — — — — — MIISCAN MIIRD
    ENC28J60_MIREGADR = 0x14U,  //  MII Register Address (MIREGADR<4:0>)
    ENC28J60_MIWRL = 0x16U, //  MII Write Data Low Byte (MIWR<7:0>)
    ENC28J60_MIWRH,         //  MII Write Data High Byte (MIWR<15:8>)
    ENC28J60_MIRDL,     // MII Read Data Low Byte (MIRD<7:0>)
    ENC28J60_MIRDH,     //  MII Read Data High Byte(MIRD<15:8>)
};

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

enum CR_BANK_3 {    // for bank 3
    ENC28J60_MAADR5,    // MAC Address Byte 5 (MAADR<15:8>)
    ENC28J60_MAADR6,    // MAC Address Byte 6 (MAADR<7:0>)
    ENC28J60_MAADR3,    // MAC Address Byte 3 (MAADR<31:24>), OUI Byte 3
    ENC28J60_MAADR4,    // MAC Address Byte 4 (MAADR<23:16>)
    ENC28J60_MAADR1,    // MAC Address Byte 1 (MAADR<47:40>), OUI Byte 1
    ENC28J60_MAADR2,    // MAC Address Byte 2 (MAADR<39:32>), OUI Byte 2
    ENC28J60_EBSTSD,    // Built-in Self-Test Fill Seed (EBSTSD<7:0>)
    ENC28J60_EBSTCON,   // PSV2     PSV1    PSV0    PSEL    TMSEL1  TMSEL0  TME     BISTST
    ENC28J60_EBSTCSL,   // Built-in Self-Test Checksum Low Byte (EBSTCS<7:0>)
    ENC28J60_EBSTCSH,   // Built-in Self-Test Checksum High Byte (EBSTCS<15:8>)
    ENC28J60_MISTAT,    // — — — — r NVALID SCAN BUSY
    ENC28J60_EREVID = 0x12U,    // Ethernet Revision ID (EREVID<4:0>) - READ ONLY REGISTER!!!
    ENC28J60_ECOCON = 0x15U,    // — — — — — COCON2 COCON1 COCON0
    ENC28J60_EFLOCON = 0x17U,   // — — — — — FULDPXS FCEN1 FCEN0
    ENC28J60_EPAUSL,    // Pause Timer Value Low Byte (EPAUS<7:0>)
    ENC28J60_EPAUSH     // Pause Timer Value High Byte (EPAUS<15:8>)
};

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

enum PHY_REGS {
    ENC28J60_PHCON1,
    ENC28J60_PHSTAT1,
    ENC28J60_PHID1,
    ENC28J60_PHID2,
    ENC28J60_PHCON2 = 0x10,
    ENC28J60_PHSTAT2,
    ENC28J60_PHIE,
    ENC28J60_PHIR,
    ENC28J60_PHLCON,
};

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

extern void enc28j60_init(uint8_t cs_num, volatile uint8_t *cs_port,
                          uint8_t rst_num, volatile uint8_t *rst_port,
                          uint8_t intr_num, volatile uint8_t * intr_port,
                          bool full_duplex);
extern void enc28j60_soft_reset(void);

extern uint8_t enc28j60_read_rev_id(void);
extern uint16_t enc28j60_read_PHY(uint8_t reg);
extern union mac_u enc28j60_get_mac(void);

#endif  /* !ENC28J60_H */
