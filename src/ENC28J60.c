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
    if (dummy)
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
static void wbm(uint8_t *buf, uint16_t count) {
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
 * @brief System reset command (soft reset)
 */
void enc28j60_soft_reset(void) {
    ether_sel();
    spi_write((7U << 5U) | 0b11111U);
    ether_desel();

    _delay_ms(0.05);    // delay 50 us after soft reset
}

/*!
 * @brief Initial ethernet shield sequence
 * @param cs_num Chip Select (SS) pin number
 * @param cs_port Chip Select port pointer
 * @param rst_num Reset (RST) pin number
 * @param rst_port Reset port pointer
 * @param intr_num Interrupt input signal pin number
 * @param intr_port Interrupt input signal port pointer
 */
void enc28j60_init(uint8_t cs_num, volatile uint8_t *cs_port,
                   uint8_t rst_num, volatile uint8_t *rst_port,
                   uint8_t intr_num, volatile uint8_t * intr_port) {
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
    _delay_ms(0.3);

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
    while (rcr(ENC28J60_ESTAT, false) & _BV(CLKRDY)) {}
    
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

    /* PHY Initialization Settings
        For proper duplex operation, the PHCON1.PDPXMD bit must also match the value of the MACON3.FULDPX bit.
        If using Half-Duplex, may to set the PHCON2.HDLDIS bit to prevent automatic loopback of the data which is transmitted.

        The PHLCON register controls the outputs of LEDA and LEDB.
        If an application requires a LED configuration other than the default,
        PHLCON must be altered to match the new requirements.
     */

}

uint8_t enc28j60_read_rev_id(void) {
    uint8_t result;

    change_memory_bank(3);
    result = rcr(ENC28J60_EREVID, false);

    return result;
}
