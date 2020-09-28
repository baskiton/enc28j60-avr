#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#include <defines.h>
#include <spi.h>

#include "ENC28J60.h"

static eth_pins ether;

#define ether_sel() (bit_clear(*(ether.cs.port), ether.cs.pin_num))
#define ether_desel() (bit_set(*(ether.cs.port), ether.cs.pin_num))

/*!
 * TODO: Move to <defines.h>
 * 
 * @brief Reversing bits on an octet.
 * Method attributed to Rich Schroeppel in the Programming Hacks section
 * http://www.inwap.com/pdp10/hbaker/hakmem/hakmem.html
 * http://www.inwap.com/pdp10/hbaker/hakmem/hacks.html#item167
 * 
 * @example:
 * 
 *   1 byte
 * _____________
 *  0100  0001
 *      ||
 *      \/
 *  1000  0010
 */
static uint8_t bits_swap(uint8_t data) {
    return (uint8_t)(((uint64_t)data * 0x0202020202U & 0x010884422010U) % 0x3ffU);
}

/*!
 * @brief Read Control Register
 * @param reg 5-bit Control Register Address
 * @param dummy If the address specifies one of the MAC or MII registers,
 * a dummy byte will first be shifted out on the SO pin.
 * @return 8-bit data from register
 */
static uint8_t rcr(uint8_t reg, bool dummy) {
    uint8_t result;

    ether_sel();
    spi_write(reg);
    if (dummy)  // dummy read
        spi_read_8();
    result = spi_read_8();
    ether_desel();

    return result;
}

/*!
 * @brief Read Buffer Memory
 * @param buf Buffer to write receiving data
 * @param count Number of bytes to read
 */
static void rbm(uint8_t *buf, uint16_t count) {
    ether_sel();
    spi_write((1U << 5U) | 0b11010U);
    spi_read_buf(buf, count);
    ether_desel();
}

/*!
 * @brief Write Control Register
 * @param reg 5-bit Control Register Address
 * @param data 8-bit write data
 */
static void wcr(uint8_t reg, uint8_t data) {
    ether_sel();
    spi_write((2U << 5U) | reg);
    spi_write(data);
    ether_desel();
}

/*!
 * @brief Write Buffer Memory
 * @param buf Data sequence to send
 * @param count Number of bytes in \p buf to write
 */
static void wbm(const uint8_t *buf, uint16_t count) {
    ether_sel();
    spi_write((3U << 5U) | 0b11010U);
    spi_write_buf(buf, count);
    ether_desel();
}

/*!
 * @brief Bit Field Set. ONLY FOR ETH CONTROL REGISTERS!!!
 * @param reg 5-bit Control Register Address
 * @param bf Bit Field to set
 */
static void bfs(uint8_t reg, uint8_t bf) {
    ether_sel();
    spi_write((4U << 5U) | reg);
    spi_write(bf);
    ether_desel();
}

/*!
 * @brief Bit Field Clear. ONLY FOR ETH CONTROL REGISTERS!!!
 * @param reg 5-bit Control Register Address
 * @param bf Bit Field to clear
 */
static void bfc(uint8_t reg, uint8_t bf) {
    ether_sel();
    spi_write((5U << 5U) | reg);
    spi_write(bf);
    ether_desel();
}

/*!
 * @brief Change the memory bank
 * @param bank Bank number from 0 to 3
 */
static void change_memory_bank(uint8_t bank) {
    if (bank > 3)
        return;
    
    uint8_t econ1;

    econ1 = rcr(ENC28J60_ECON1, false);
    econ1 = (econ1 & 0b11111100U) | bank;
    wcr(ENC28J60_ECON1, econ1);
}

/*!
 * @brief Reading PHY register
 * @param reg PHY Register Address
 * @return 16-bit data from \p reg
 */
static uint16_t phy_read(uint8_t reg) {
    uint16_t result = 0;

    change_memory_bank(2);
    wcr(ENC28J60_MIREGADR, reg);
    bfs(ENC28J60_MICMD, _BV(MIIRD));    // starting read
    _delay_us(10.24);
    change_memory_bank(3);
    while (rcr(ENC28J60_MISTAT, true) & _BV(BUSY)) {}
    change_memory_bank(2);
    bfc(ENC28J60_MICMD, _BV(MIIRD));
    result = rcr(ENC28J60_MIRDH, true) << 8U;
    result |= rcr(ENC28J60_MIRDL, true);

    return result;
}

/*!
 * @brief Writing PHY register.
 * @param reg PHY Register Address
 * @param data 16-bit write data
 */
static void phy_write(uint8_t reg, uint16_t data) {
    change_memory_bank(2);
    wcr(ENC28J60_MIREGADR, reg);
    wcr(ENC28J60_MIWRL, (uint8_t)(data & 0xFF));
    wcr(ENC28J60_MIWRH, (uint8_t)(data >> 8));  // starting write
    _delay_us(10.24);
    change_memory_bank(3);
    while (rcr(ENC28J60_MISTAT, true) & _BV(BUSY)) {}
}

/*!
 * @brief Automatic scanning specific PHY register.
 * Data will be written to the \p MIRD registers every 10.24 μs.
 * When the \p MIISCAN operation is in progress, the host controller
 * must not attempt to write to \p MIWRH or start an \p MIIRD operation.
 * @param reg PHY Register Address
 */
static void phy_scan_start(uint8_t reg) {
    change_memory_bank(2);
    wcr(ENC28J60_MIREGADR, reg);
    bfs(ENC28J60_MICMD, _BV(MIISCAN));  // starting scan
}

/*!
 * @brief Stop automatically scanning of PHY register
 */
static void phy_scan_stop(void) {
    change_memory_bank(2);
    bfc(ENC28J60_MICMD, _BV(MIISCAN));
    change_memory_bank(3);
    while (rcr(ENC28J60_MISTAT, true) & _BV(BUSY)) {}
}

/*!
 * @brief Get value with autoscan PHY-gerister.
 * Since the host controller can only read one MII register at a time,
 * the \p MIRDL and \p MIRDH values ​​will be read from the PHY at different times.
 * @return 16-bit data from PHY-register
 */
static uint16_t phy_scan_rd(void) {
    uint16_t result = 0;

    change_memory_bank(3);
    while (rcr(ENC28J60_MISTAT, true) & _BV(NVALID)) {}

    change_memory_bank(2);
    result = rcr(ENC28J60_MIRDH, true) << 8U;
    result |= rcr(ENC28J60_MIRDL, true);

    return result;
}

/*!
 * @brief Get OUI from PHY-registers and making it canonical
 * @return Canonical OUI
 */
static uint32_t get_oui(void) {
    uint32_t oui, tmp;

    tmp = phy_read(ENC28J60_PHID2) >> 10;
    tmp |= ((uint32_t)phy_read(ENC28J60_PHID1) << 6);

    oui = (uint32_t)bits_swap((uint8_t)(tmp & 0x3FU));
    oui |= (uint32_t)bits_swap((uint8_t)((tmp >> 8U) & 0xFFU)) << 8U;
    oui |= (uint32_t)bits_swap((uint8_t)((tmp >> 16U) & 0xFFU)) << 16U;

    return oui;
}

/*!
 * @brief System reset command (soft reset)
 */
void enc28j60_soft_reset(void) {
    ether_sel();
    spi_write((7U << 5U) | 0b11111U);
    ether_desel();

    _delay_us(50);    // delay 50 us after soft reset
}

/*!
 * @brief Initial ethernet shield sequence
 * @param cs_num Chip Select (SS) pin number
 * @param cs_port Chip Select port pointer
 * @param rst_num Reset (RST) pin number
 * @param rst_port Reset port pointer
 * @param intr_num Interrupt input signal pin number
 * @param intr_port Interrupt input signal port pointer
 * @param full_duplex \c true - Full-Duplex mode using;
 *                    \c false - Half-Duplex mode using
 */
void enc28j60_init(uint8_t cs_num, volatile uint8_t *cs_port,
                   uint8_t rst_num, volatile uint8_t *rst_port,
                   uint8_t intr_num, volatile uint8_t * intr_port,
                   bool full_duplex) {
    union mac_u mac_addr;

    ether.cs.pin_num = cs_num;
    ether.cs.port = cs_port;
    ether.rst.pin_num = rst_num;
    ether.rst.port = rst_port;
    ether.intr.pin_num = intr_num;
    ether.intr.port = intr_port;
    
    set_output(*(ether.cs.port - 1), ether.cs.pin_num);
    // set_output(*(port - 1), rst);
    set_input(*(ether.intr.port - 1), ether.intr.pin_num);
    
    spi_set_speed(ETHER_FREQ);
    
    /* Oscillator Start-up Timer (OST) - 300 us delay after power-up */
    _delay_us(300);

    /* reset routine */
    enc28j60_soft_reset();

    /* INITIALIZATION */
    
    /* Receive Buffer
       Set ERXST and ERXND pointer - receive buffer. For example: 0x0000 - 0x0FFF.
       For tracking, the ERXRDPT registers should be programmed with the same value
       (ERXRDPTL first, followed by ERXRDPTH).
     */
    change_memory_bank(0);
    wcr(ENC28J60_ERXSTL, 0);    // ERXST = 0x0000
    wcr(ENC28J60_ERXSTH, 0);
    wcr(ENC28J60_ERXNDL, 0xFF); // ERXND = 0x0FFF
    wcr(ENC28J60_ERXNDH, 0x0F);
    wcr(ENC28J60_ERXRDPTL, 0);  // ERXRDPT = ERXST = 0x0000
    wcr(ENC28J60_ERXRDPTH, 0);
    
    /* Transmit Buffer
       No explicit action is required to initialize the transmission buffer.
     */

    /* Receive Filters
       The appropriate receive filters should be enabled or disabled by writing to the ERXFCON register.
       By default is set UCEN, CRCEN, BCEN
     */
    change_memory_bank(1);
    wcr(ENC28J60_ERXFCON, (_BV(UCEN) | _BV(CRCEN) | _BV(BCEN)));

    /* Waiting for OST
       CLKRDY bit must be polled before
       transmitting packets, enabling packet
       reception or accessing any MAC, MII or
       PHY registers.
     */
    while (!(rcr(ENC28J60_ESTAT, false) & _BV(CLKRDY))) {}
    
    /* MAC Initialization Settings
        1. Set the MARXEN bit in MACON1 to enable the MAC to receive frames.
            If using full duplex, also set TXPAUS and RXPAUS.
        2. Configure the PADCFG (101), TXCRCEN and FULDPX bits of MACON3.
            Set the FRMLNEN bit to enable frame length status reporting.
            The FULDPX bit should be set if the app will be connected to a
            full-duplex configured remote node; otherwise, it should be left clear.
        3. Configure the bits in MACON4. For conformance to the IEEE 802.3 standard, set the DEFER bit.
        4. Program the MAMXFL registers with the maximum frame length to be
            permitted to be received or transmitted. Normal network nodes are
            designed to handle packets that are 1518 bytes or less.
        5. Configure the Back-to-Back Inter-Packet Gap register, MABBIPG.
            0x15 for Full-Duplex mode;
            0x12 for Half-Duplex mode.
        6. Configure the Non-Back-to-Back Inter-Packet Gap register low byte, MAIPGL.
            Most applications will program this register with 0x12.
        7. If Half-Duplex is used, the MAIPGH should be programmed.
            Most applications will program this register to 0x0C.
        8. If Half-Duplex is used, program the Retransmission and
            Collision Window registers, MACLCON1 and MACLCON2.
            Most applications will not need to change the default Reset values.
            If the network is spread over exceptionally long cables,
            the default value of MACLCON2 may need to be increased.
        9. Program the local MAC address into the MAADR1:MAADR6 registers.
     */
    change_memory_bank(2);
    if (full_duplex) {
        wcr(ENC28J60_MACON1, (_BV(MARXEN) | _BV(TXPAUS) | _BV(RXPAUS)));    // 1.
        wcr(ENC28J60_MACON3, (_BV(PADCFG2) | _BV(PADCFG0) | _BV(TXCRCEN) | _BV(FRMLNEN) | _BV(FULDPX)));    // 2.
        wcr(ENC28J60_MACON4, _BV(DEFER));   // 3.
        wcr(ENC28J60_MABBIPG, 0x15U);   // 5.
    } else {
        wcr(ENC28J60_MACON1, _BV(MARXEN));  // 1.
        wcr(ENC28J60_MACON3, (_BV(PADCFG2) | _BV(PADCFG0) | _BV(TXCRCEN) | _BV(FRMLNEN)));  // 2.
        wcr(ENC28J60_MACON4, (_BV(DEFER) | _BV(BPEN) | _BV(NOBKOFF)));  // 3.
        wcr(ENC28J60_MABBIPG, 0x12U);   // 5.
        wcr(ENC28J60_MAIPGH, 0x0CU);    // 7.
        wcr(ENC28J60_MACLCON1, 0x0FU);  // 8.   defaul value
        wcr(ENC28J60_MACLCON2, 0x37U);  // 8.   defaul value
    }

    wcr(ENC28J60_MAMXFLL, (ETHER_MAX_FRAME_LEN & 0xFFU)); // 4.
    wcr(ENC28J60_MAMXFLH, (ETHER_MAX_FRAME_LEN >> 8U));   // MAMXFL = 1518
    wcr(ENC28J60_MAIPGL, 0x12U);    // 6.

    mac_addr.mac_64 = (uint64_t)get_oui() << 24U;

    change_memory_bank(3);  // 9.
    wcr(ENC28J60_MAADR1, mac_addr.mac_s.mac_1);
    wcr(ENC28J60_MAADR2, mac_addr.mac_s.mac_2);
    wcr(ENC28J60_MAADR3, mac_addr.mac_s.mac_3);
    wcr(ENC28J60_MAADR4, mac_addr.mac_s.mac_4);
    wcr(ENC28J60_MAADR5, mac_addr.mac_s.mac_5);
    wcr(ENC28J60_MAADR6, mac_addr.mac_s.mac_6);

    /* PHY Initialization Settings
        For proper duplex operation, the PHCON1.PDPXMD bit must
        also match the value of the MACON3.FULDPX bit.
        If using Half-Duplex, may to set the PHCON2.HDLDIS bit to
        prevent automatic loopback of the data which is transmitted.

        The PHLCON register controls the outputs of LEDA and LEDB.
        If an application requires a LED configuration other than the default,
        PHLCON must be altered to match the new requirements.
     */
    if (full_duplex)
        phy_write(ENC28J60_PHCON1, (phy_read(ENC28J60_PHCON1) | _BV(PDPXMD)));
    else {
        phy_write(ENC28J60_PHCON1, (phy_read(ENC28J60_PHCON1) & ~_BV(PDPXMD)));
        phy_write(ENC28J60_PHCON2, (phy_read(ENC28J60_PHCON2) | _BV(HDLDIS)));
    }
}

/*!
 * @brief Get Ethernet Revision ID
 * @return 5-bit revision ID
 */
uint8_t enc28j60_read_rev_id(void) {
    uint8_t result;

    change_memory_bank(3);
    result = rcr(ENC28J60_EREVID, false) & 0x1FU;

    return result;
}

/*!
 * @brief Get PHY register value
 * @param reg PHY register address
 * @return 16-bit data
 */
uint16_t enc28j60_read_PHY(uint8_t reg) {
    return phy_read(reg);
}

/*!
 * @brief Get MAC-address
 * @return MAC-addr in \c union
 */
union mac_u enc28j60_get_mac(void) {
    union mac_u mac;

    change_memory_bank(3);
    mac.mac_s.mac_1 = rcr(ENC28J60_MAADR1, true);
    mac.mac_s.mac_2 = rcr(ENC28J60_MAADR2, true);
    mac.mac_s.mac_3 = rcr(ENC28J60_MAADR3, true);
    mac.mac_s.mac_4 = rcr(ENC28J60_MAADR4, true);
    mac.mac_s.mac_5 = rcr(ENC28J60_MAADR5, true);
    mac.mac_s.mac_6 = rcr(ENC28J60_MAADR6, true);

    return mac;
}

/*!
 * @brief Transmitting Packet
 * @param mac_dest Destination MAC address
 * @param mac_src Source MAC address
 * @param type_len EtherType or length of the packet (amount of non-padding data; 1500 or less)
 * @param data Data Packet Payload (0 - 1500 bytes)
 * @param data_len Lenght of \p data buffer
 * @param ppcb Per Packet Control Byte. 0 by default. Otherwise, refer to the datasheet on chapter 7.1
 */
void enc28j60_packet_transmit(const union mac_u *mac_dest, const union mac_u *mac_src,
                              const uint16_t *type_len, const uint8_t *data, uint16_t data_len,
                              uint8_t ppcb) {
    /* check if transfer is in progress
     */
    while (rcr(ENC28J60_ECON1, false) & _BV(TXRTS)) {}
    
    /* set ETXST. It is recommended that an even address be used
     */
    
    wbm(&ppcb, 1);

    wbm(&mac_dest->mac_s.mac_1, 1);
    wbm(&mac_dest->mac_s.mac_2, 1);
    wbm(&mac_dest->mac_s.mac_3, 1);
    wbm(&mac_dest->mac_s.mac_4, 1);
    wbm(&mac_dest->mac_s.mac_5, 1);
    wbm(&mac_dest->mac_s.mac_6, 1);
    
    wbm(&mac_src->mac_s.mac_1, 1);
    wbm(&mac_src->mac_s.mac_2, 1);
    wbm(&mac_src->mac_s.mac_3, 1);
    wbm(&mac_src->mac_s.mac_4, 1);
    wbm(&mac_src->mac_s.mac_5, 1);
    wbm(&mac_src->mac_s.mac_6, 1);

    wbm((uint8_t *)type_len + 1, 1);
    wbm((uint8_t *)type_len, 1);

    wbm(data, ((data_len > 1500) ? 1500 : data_len));

    /* set ETXND. It should point to the last byte in the data payload.
    */

    bfc(ENC28J60_EIR, _BV(TXIF));
    bfs(ENC28J60_EIE, _BV(TXIE));
    // bfs(ENC28J60_EIE, _BV(INTIE));  // set EIE.INTIE to enable an interrupt when done (if desired).

    bfs(ENC28J60_ECON1, _BV(TXRTS));    // Start the transmission process
}

/*!
 * @brief Receiving Packet
 */
void enc28j60_packet_receive(void) {

}
