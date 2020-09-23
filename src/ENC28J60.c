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
    
    /*
    CLKRDY bit must be polled before
    transmitting packets, enabling packet
    reception or accessing any MAC, MII or
    PHY registers. */
    
    
}

uint8_t enc28j60_read_rev_id(void) {
    uint8_t result;

    change_memory_bank(3);
    result = rcr(ENC28J60_EREVID, false);

    return result;
}
