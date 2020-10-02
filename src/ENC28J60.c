#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <defines.h>
#include <spi.h>

#include "ENC28J60.h"

#define ether_sel() (bit_clear(*(enc28j60.spi_dev.cs.port), enc28j60.spi_dev.cs.pin_num))
#define ether_desel() (bit_set(*(enc28j60.spi_dev.cs.port), enc28j60.spi_dev.cs.pin_num))

static struct enc28j60_dev {
    struct spi_device_s spi_dev;
    uint16_t next_packet_ptr;
    void (*rx_handler)(uint8_t *buf, uint16_t size, uint8_t step);
} enc28j60;

static uint8_t rcr(uint8_t addr);
static void wcr(uint8_t addr, uint8_t data);

/*!
 * TODO: Move to <defines.h>
 * 
 * @brief Reversing bits on an octet.
 * Method attributed to Rich Schroeppel in the Programming Hacks section
 * http://www.inwap.com/pdp10/hbaker/hakmem/hakmem.html
 * http://www.inwap.com/pdp10/hbaker/hakmem/hacks.html#item167
 * 
 * @example:
 *   1 byte
 * ____________
 *  0110  0001
 *      ||
 *      \/
 *  1000  0110
 */
static uint8_t bits_swap(uint8_t data) {
    return (uint8_t)(((uint64_t)data * 0x0202020202U & 0x010884422010U) % 0x3ffU);
}

/*!
 * @brief Change the memory bank
 * @param addr Register address
 */
static void change_memory_bank(uint8_t addr) {
    // return if it is Common Register
    if ((addr >= ENC28J60_EIE) && (addr <= ENC28J60_ECON1))
        return;

    uint8_t bank = (addr & ENC28J60_BANK_MASK) >> 5;
    
    uint8_t econ1;

    econ1 = rcr(ENC28J60_ECON1);
    econ1 = (econ1 & 0b11111100U) | bank;
    wcr(ENC28J60_ECON1, econ1);
}

/*!
 * @brief Read Control Register
 * @param addr Control Register Address
 * @param dummy If the address specifies one of the MAC or MII registers,
 * a dummy byte will first be shifted out on the SO pin.
 * @return 8-bit data from register
 */
static uint8_t rcr(uint8_t addr) {
    uint8_t result;

    change_memory_bank(addr);
    
    ether_sel();
    spi_write(ENC28J60_RCR_OP | (addr & ENC28J60_ADDR_MASK));
    if (addr & ENC28J60_DUMMY_MASK)  // dummy read
        spi_read_8();
    result = spi_read_8();
    ether_desel();

    return result;
}

/*!
 * @brief Read Buffer Memory
 * @param buf Buffer to write receiving data
 * @param count Number of bytes to read
 * @param addr Address of start reading
 */
static void rbm(uint8_t *buf, uint16_t count, uint16_t addr) {
    wcr(ENC28J60_ERDPTL, (uint8_t)addr);
    wcr(ENC28J60_ERDPTH, (uint8_t)(addr >> 8));

    ether_sel();
    spi_write(ENC28J60_RBM_OP);
    spi_read_buf(buf, count);
    ether_desel();
}

/*!
 * @brief Write Control Register
 * @param addr Control Register Address
 * @param data 8-bit write data
 */
static void wcr(uint8_t addr, uint8_t data) {
    change_memory_bank(addr);

    ether_sel();
    spi_write(ENC28J60_WCR_OP | (addr & ENC28J60_ADDR_MASK));
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
    spi_write(ENC28J60_WBM_OP);
    spi_write_buf(buf, count);
    ether_desel();
}

/*!
 * @brief Bit Field Set. ONLY FOR ETH CONTROL REGISTERS!!!
 * @param addr Control Register Address
 * @param bf Bit Field to set
 */
static void bfs(uint8_t addr, uint8_t bf) {
    change_memory_bank(addr);

    ether_sel();
    spi_write(ENC28J60_BFS_OP | (addr & ENC28J60_ADDR_MASK));
    spi_write(bf);
    ether_desel();
}

/*!
 * @brief Bit Field Clear. ONLY FOR ETH CONTROL REGISTERS!!!
 * @param addr 5-bit Control Register Address
 * @param bf Bit Field to clear
 */
static void bfc(uint8_t addr, uint8_t bf) {
    change_memory_bank(addr);

    ether_sel();
    spi_write(ENC28J60_BFC_OP | (addr & ENC28J60_ADDR_MASK));
    spi_write(bf);
    ether_desel();
}

/*!
 * @brief Reading PHY register
 * @param reg PHY Register Address
 * @return 16-bit data from \p reg
 */
static uint16_t phy_read(uint8_t reg) {
    uint16_t result = 0;

    wcr(ENC28J60_MIREGADR, reg);
    bfs(ENC28J60_MICMD, _BV(MIIRD));    // starting read
    _delay_us(10.24);
    while (rcr(ENC28J60_MISTAT) & _BV(BUSY)) {}
    bfc(ENC28J60_MICMD, _BV(MIIRD));
    result = rcr(ENC28J60_MIRDH) << 8;
    result |= rcr(ENC28J60_MIRDL);

    return result;
}

/*!
 * @brief Writing PHY register.
 * @param reg PHY Register Address
 * @param data 16-bit write data
 */
static void phy_write(uint8_t reg, uint16_t data) {
    wcr(ENC28J60_MIREGADR, reg);
    wcr(ENC28J60_MIWRL, (uint8_t)(data & 0xFF));
    wcr(ENC28J60_MIWRH, (uint8_t)(data >> 8));  // starting write
    _delay_us(10.24);
    while (rcr(ENC28J60_MISTAT) & _BV(BUSY)) {}
}

/*!
 * @brief Automatic scanning specific PHY register.
 * Data will be written to the \p MIRD registers every 10.24 μs.
 * When the \p MIISCAN operation is in progress, the host controller
 * must not attempt to write to \p MIWRH or start an \p MIIRD operation.
 * @param reg PHY Register Address
 */
static void phy_scan_start(uint8_t reg) {
    wcr(ENC28J60_MIREGADR, reg);
    bfs(ENC28J60_MICMD, _BV(MIISCAN));  // starting scan
}

/*!
 * @brief Stop automatically scanning of PHY register
 */
static void phy_scan_stop(void) {
    bfc(ENC28J60_MICMD, _BV(MIISCAN));
    while (rcr(ENC28J60_MISTAT) & _BV(BUSY)) {}
}

/*!
 * @brief Get value with autoscan PHY-gerister.
 * Since the host controller can only read one MII register at a time,
 * the \p MIRDL and \p MIRDH values ​​will be read from the PHY at different times.
 * @return 16-bit data from PHY-register
 */
static uint16_t phy_scan_rd(void) {
    uint16_t result = 0;

    while (rcr(ENC28J60_MISTAT) & _BV(NVALID)) {}

    result = rcr(ENC28J60_MIRDH) << 8;
    result |= rcr(ENC28J60_MIRDL);

    return result;
}

/*!
 * @brief Get OUI from PHY-registers and making it canonical
 * @param oui 3-bytes OUI buffer to store
 */
static void get_oui(uint8_t *oui) {
    uint32_t tmp;

    tmp = phy_read(ENC28J60_PHID2) >> 10;
    tmp |= ((uint32_t)phy_read(ENC28J60_PHID1) << 6);

    oui[0] = bits_swap(((tmp >> 16) & 0xFFU));
    oui[1] = bits_swap(((tmp >> 8) & 0xFFU));
    oui[2] = bits_swap((tmp & 0xFFU));
}

/*!
 * ERXRDPT need to be set always at odd addresses
 * (by ERRATA, issue #14)
 */
static uint16_t erxrdpt_workaround(uint16_t next_packet_ptr, uint16_t rxst, uint16_t rxnd) {
    if ((next_packet_ptr == rxst) || ((next_packet_ptr - 1) > rxnd))
        return rxnd;
    return (next_packet_ptr - 1);
}

/*!
 * @brief RX buffer initialize.
 *  Set ERXST and ERXND pointer - receive buffer.
 *  For tracking, the ERXRDPT registers should be programmed with the same value
 *  (ERXRDPTL first, followed by ERXRDPTH).
 * @param start Start address of RX buffer
 * @param end End address of RX buffer
 */
static void rx_buf_init(uint16_t start, uint16_t end) {
    uint16_t erxrdpt;

    if ((start > 0x1FFF) || (end > 0x1FFF) || (start > end)) {
        printf_P(PSTR("rx_buf_init(%d, %d) - RX buf bad parameters!\n"), start, end);
        return;
    }
    
    enc28j60.next_packet_ptr = start;
    wcr(ENC28J60_ERXSTL, (uint8_t)start);
    wcr(ENC28J60_ERXSTH, (uint8_t)(start >> 8));

    wcr(ENC28J60_ERXNDL, (uint8_t)end);
    wcr(ENC28J60_ERXNDH, (uint8_t)(end >> 8));

    erxrdpt = erxrdpt_workaround(enc28j60.next_packet_ptr, start, end);
    wcr(ENC28J60_ERXRDPTL, (uint8_t)erxrdpt);
    wcr(ENC28J60_ERXRDPTH, (uint8_t)(erxrdpt >> 8));
}

/*!
 * @brief TX buffer initialize.
 * ETXST and ATXND will not be changed after transmit operation.
 * @param start Start address of TX buffer
 * @param end End address of TX buffer
 */
static void tx_buf_init(uint16_t start, uint16_t end) {
    if ((start > 0x1FFF) || (end > 0x1FFF) || (start > end)) {
        printf_P(PSTR("tx_buf_init(%d, %d) - TX buf bad parameters!\n"), start, end);
        return;
    }
    
    wcr(ENC28J60_ETXSTL, (uint8_t)start);
    wcr(ENC28J60_ETXSTH, (uint8_t)(start >> 8));

    wcr(ENC28J60_ETXNDL, (uint8_t)end);
    wcr(ENC28J60_ETXNDH, (uint8_t)(end >> 8));
}

/*!
 * @brief Calculate random access address of RX buffer
 * @param addr Packet start address
 * @param offset Offset
 * @return Computed address
 */
static uint16_t rand_acc_addr_calc(uint16_t addr, uint16_t offset) {
    uint16_t tmp = addr + offset;
    
    if (tmp > ENC28J60_RXEND_INIT)
        return (tmp - (ENC28J60_RXEND_INIT - ENC28J60_RXSTART_INIT + 1));
    return tmp;
}

/*!
 * @brief Enable chip to RX and TX, enable interrupts
 */
void enc28j60_enable(void) {
    /* enable interrupt logic */

    phy_write(ENC28J60_PHIR, 0);    // clear all PHY interrupt flags
    phy_write(ENC28J60_PHIE, (_BV(PGEIE) | _BV(PLNKIE)));

    wcr(ENC28J60_EIR, 0);   // clear all interrupt flags
    bfs(ENC28J60_EIE, (_BV(INTIE) | _BV(PKTIE) | _BV(DMAIE) | _BV(LINKIE) |
                       _BV(TXIE) | _BV(TXERIF) | _BV(RXERIE)));

    /* enable receive */
    bfs(ENC28J60_ECON1, _BV(RXEN));
}

/*!
 * @brief Disable chip interrupts and packet reception
 */
void enc28j60_disable(void) {
    // disable all interrupts
    phy_write(ENC28J60_PHIE, 0);
    wcr(ENC28J60_EIE, 0);
    
    // disable packet reception
    bfc(ENC28J60_ECON1, _BV(RXEN));
}

/*!
 * @brief System reset command (soft reset)
 */
void enc28j60_soft_reset(void) {
    ether_sel();
    spi_write(ENC28J60_SRC_OP);
    ether_desel();

    // _delay_us(50);    // delay 50 us after soft reset
    _delay_us(1000);    // delay at least 1 ms. Errata issue #2
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
 * @param rx_handler Function pointer of the RX packet handler
 */
void enc28j60_init(uint8_t cs_num, volatile uint8_t *cs_port,
                   uint8_t rst_num, volatile uint8_t *rst_port,
                   uint8_t intr_num, volatile uint8_t * intr_port,
                   bool full_duplex, void (*rx_handler)(uint8_t *, uint16_t, uint8_t)) {
    uint8_t oui[3];

    enc28j60_disable();

    enc28j60.spi_dev.cs.pin_num = cs_num;
    enc28j60.spi_dev.cs.port = cs_port;
    enc28j60.spi_dev.rst.pin_num = rst_num;
    enc28j60.spi_dev.rst.port = rst_port;
    enc28j60.spi_dev.intr.pin_num = intr_num;
    enc28j60.spi_dev.intr.port = intr_port;
    enc28j60.spi_dev.a0.pin_num = 0;
    enc28j60.spi_dev.a0.port = NULL;
    enc28j60.rx_handler = rx_handler;
    
    set_output(*(enc28j60.spi_dev.cs.port - 1), enc28j60.spi_dev.cs.pin_num);
    set_input(*(enc28j60.spi_dev.intr.port - 1), enc28j60.spi_dev.intr.pin_num);
    
    spi_set_speed(ENC28J60_MAX_FREQ);
    
    /* Oscillator Start-up Timer (OST) - 300 us delay after power-up */
    _delay_us(300);

    /* reset routine */
    enc28j60_soft_reset();

    /* INITIALIZATION */
    uint8_t revid = rcr(ENC28J60_EREVID) & 0x1F;
    if ((revid == 0x00) || (revid == 0xFFU)) {
        printf_P(PSTR("enc28j60_init() - Invalid REVID %d\n"), revid);
        return;
    }
    
    /* Receive Buffer init*/
    rx_buf_init(ENC28J60_RXSTART_INIT, ENC28J60_RXEND_INIT);
    
    /* Transmit Buffer init */
    tx_buf_init(ENC28J60_TXSTART_INIT, ENC28J60_TXEND_INIT);

    /* Receive Filters
       The appropriate receive filters should be enabled or disabled by writing to the ERXFCON register.
       By default is set UCEN, CRCEN, BCEN
     */
    wcr(ENC28J60_ERXFCON, (_BV(UCEN) | _BV(CRCEN) | _BV(BCEN)));

    /* Waiting for OST
       CLKRDY bit must be polled before
       transmitting packets, enabling packet
       reception or accessing any MAC, MII or
       PHY registers.
     */
    while (!(rcr(ENC28J60_ESTAT) & _BV(CLKRDY))) {}
    
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
    if (full_duplex) {
        wcr(ENC28J60_MACON1, (_BV(MARXEN) | _BV(TXPAUS) | _BV(RXPAUS)));    // 1.
        wcr(ENC28J60_MACON3, (_BV(PADCFG2) | _BV(PADCFG0) | _BV(TXCRCEN) | _BV(FRMLNEN) | _BV(FULDPX)));    // 2.
        wcr(ENC28J60_MACON4, _BV(DEFER));   // 3.
        wcr(ENC28J60_MABBIPG, 0x15);   // 5.
    } else {
        wcr(ENC28J60_MACON1, _BV(MARXEN));  // 1.
        wcr(ENC28J60_MACON3, (_BV(PADCFG2) | _BV(PADCFG0) | _BV(TXCRCEN) | _BV(FRMLNEN)));  // 2.
        wcr(ENC28J60_MACON4, (_BV(DEFER) | _BV(BPEN) | _BV(NOBKOFF)));  // 3.
        wcr(ENC28J60_MABBIPG, 0x12);   // 5.
        wcr(ENC28J60_MAIPGH, 0x0C);    // 7.
        wcr(ENC28J60_MACLCON1, 0x0F);  // 8.   defaul value
        wcr(ENC28J60_MACLCON2, 0x37);  // 8.   defaul value
    }

    wcr(ENC28J60_MAMXFLL, (uint8_t)(ENC28J60_MAX_FRAME_LEN & 0xFFU));   // 4.
    wcr(ENC28J60_MAMXFLH, (uint8_t)(ENC28J60_MAX_FRAME_LEN >> 8));     // MAMXFL = 1518
    wcr(ENC28J60_MAIPGL, 0x12);    // 6.

    get_oui(oui);
    
    // 9.
    wcr(ENC28J60_MAADR1, oui[0]);
    wcr(ENC28J60_MAADR2, oui[1]);
    wcr(ENC28J60_MAADR3, oui[2]);
    wcr(ENC28J60_MAADR4, 0);
    wcr(ENC28J60_MAADR5, 0);
    wcr(ENC28J60_MAADR6, 0);

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

    printf_P(PSTR("ENC28J60 initialized with RevID %d\n"), revid);

    enc28j60_enable();
}

/*!
 * @brief Get Ethernet Revision ID
 * @return 5-bit revision ID
 */
uint8_t enc28j60_read_rev_id(void) {
    uint8_t result;

    result = rcr(ENC28J60_EREVID) & 0x1F;

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
 * @param mac_buf Buffer to store MAC-addr.
 * Make sure that buffer is long enough (default 6 bytes).
 */
void enc28j60_get_mac(uint8_t *mac_buf) {
    mac_buf[0] = rcr(ENC28J60_MAADR1);
    mac_buf[1] = rcr(ENC28J60_MAADR2);
    mac_buf[2] = rcr(ENC28J60_MAADR3);
    mac_buf[3] = rcr(ENC28J60_MAADR4);
    mac_buf[4] = rcr(ENC28J60_MAADR5);
    mac_buf[5] = rcr(ENC28J60_MAADR6);
}

/*!
 * @brief Transmitting Packet
 * @param frame OSI Layer 2 Ethernet frame
 * @param len Lenght of \p frame buffer (max 1514)
 * @param ppcb Per Packet Control Byte. 0 by default. Otherwise, refer to the datasheet on chapter 7.1
 */
void enc28j60_packet_transmit(const uint8_t *frame, uint16_t len, uint8_t ppcb) {
    uint16_t end;

    /* check if transfer is in progress. Needed? */
    while (rcr(ENC28J60_ECON1) & _BV(TXRTS)) {}
    
    /* set EWRPT - pointer to start of transmit buffer */
    wcr(ENC28J60_EWRPTL, (uint8_t)ENC28J60_TXSTART_INIT);
    wcr(ENC28J60_EWRPTH, (uint8_t)(ENC28J60_TXSTART_INIT >> 8));
    
    /* write to buffer */
    wbm(&ppcb, 1);
    wbm(frame, len);

    /* set ETXND. It should point to the last byte in the data payload. */
    end = ENC28J60_TXSTART_INIT + len;
    wcr(ENC28J60_ETXNDL, (uint8_t)end);
    wcr(ENC28J60_ETXNDH, (uint8_t)(end >> 8));

    /* Clear EIR.TXIF. Needed? */
    bfc(ENC28J60_EIR, _BV(TXIF));

    /* Start the transmission process */
    bfs(ENC28J60_ECON1, _BV(TXRTS));
}

/*!
 * @brief Receiving Packet
 */
void enc28j60_packet_receive(void) {
    uint8_t rsv[ENC28J60_RSV_SIZE]; // receive status vector
    uint16_t next_packet_ptr, packet_len, rx_status, erxrdpt, size_tmp, offset;
    uint8_t *data, step_num;

    /* reading receive status vector */
    rbm(rsv, ENC28J60_RSV_SIZE, enc28j60.next_packet_ptr);
    next_packet_ptr = (rsv[1] << 8) | rsv[0];
    packet_len = (rsv[3] << 8) | rsv[2];
    rx_status = (rsv[5] << 8) | rsv[4];
    
    if (!(rx_status & _BV(RSV_RX_OK)) || (packet_len > ENC28J60_MAX_FRAME_LEN)) {
        printf_P(PSTR("--- Receive Packet Error!  ---\n"));
        // error - CRC or Len Check
        if (rx_status & _BV(RSV_CRC_ERR))
            printf_P(PSTR("--- --- CRC Error! --- --- ---\n"));
        if (rx_status & _BV(RSV_LEN_CHECK_ERR))
            printf_P(PSTR("--- --- Lenght Check Error! --\n"));
        if (packet_len > ENC28J60_MAX_FRAME_LEN)
            printf_P(PSTR("--- --- Max Lenght Error - %u\n"), packet_len);
        next_packet_ptr = enc28j60.next_packet_ptr;
    } else {
        /* reading received packets */
        offset = ENC28J60_RSV_SIZE; // offset from the packet pointer
        step_num = 0;
        while (packet_len) {
            // division into several steps with a maximum of 512 bytes
            if (packet_len > 0x0200U) {
                size_tmp = 0x0200;
                packet_len -= 0x0200;
            } else {
                size_tmp = packet_len;
                packet_len = 0;
            }
            
            data = malloc(size_tmp);
            if (data == NULL) {
                // what to do if not enough memory?
                printf_P(PSTR("Out of memory for RX buffer - %d bytes\n"), size_tmp);
                break;
            }
            rbm(data, size_tmp, rand_acc_addr_calc(enc28j60.next_packet_ptr, offset));
            enc28j60.rx_handler(data, size_tmp, step_num++);    // handling
            free(data);
            offset += size_tmp;
        }
    }

    /* freeing receive buffer space */
    erxrdpt = erxrdpt_workaround(next_packet_ptr, ENC28J60_RXSTART_INIT, ENC28J60_RXEND_INIT);
    wcr(ENC28J60_ERXRDPTL, (uint8_t)erxrdpt);
    wcr(ENC28J60_ERXRDPTH, (uint8_t)(erxrdpt >> 8));

    /** At this moment \a rand_acc_addr_calc(enc28j60.next_packet_ptr,offset)
     * must be equal to \a next_packet_ptr */
    enc28j60.next_packet_ptr = next_packet_ptr;
    bfs(ENC28J60_ECON2, _BV(PKTDEC));   // decrement packet count
}

/*!
 * @brief Get size of free space of RX buffer
 * @return Free space in the RX buffer
 */
int16_t enc28j60_get_rx_free_space(void) {
    uint8_t epktcnt;
    uint16_t erxwrpt, erxrdpt, erxst, erxnd;
    int16_t result;

    epktcnt = rcr(ENC28J60_EPKTCNT);
    if (epktcnt >= 255)
        return -1;
    
    while (true) {
        erxwrpt = (uint16_t)rcr(ENC28J60_ERXWRPTL);
        erxwrpt |= (uint16_t)rcr(ENC28J60_ERXWRPTH) << 8;
        if (epktcnt == rcr(ENC28J60_EPKTCNT))
            break;
    }

    erxrdpt = (uint16_t)rcr(ENC28J60_ERXRDPTL);
    erxrdpt |= (uint16_t)rcr(ENC28J60_ERXRDPTH) << 8;

    erxst = (uint16_t)rcr(ENC28J60_ERXSTL);
    erxst |= (uint16_t)rcr(ENC28J60_ERXSTH) << 8;

    erxnd = (uint16_t)rcr(ENC28J60_ERXNDL);
    erxnd |= (uint16_t)rcr(ENC28J60_ERXNDH) << 8;

    if (erxwrpt > erxrdpt)
        result = (erxnd - erxst) - (erxwrpt - erxrdpt);
    else if (erxwrpt == erxrdpt)
        result = erxnd - erxst;
    else
        result = erxrdpt - erxwrpt - 1;

    return result;
}

/*!
 * @brief Check Link Status
 * @return True if Link Up; False otherwise
 */
bool check_link(void) {
    return (enc28j60_read_PHY(ENC28J60_PHSTAT2) & _BV(LSTAT)) >> LSTAT;
}

/*!
 * @brief Interrupt Request Handler
 */
void enc28j60_irq_handler(void) {
    uint8_t intrs;

    bfc(ENC28J60_EIE, _BV(INTIE));  // disable interrupts
    printf_P(PSTR("\nInterrupt has occurred:\n"));

    intrs = rcr(ENC28J60_EIR);

    /* Receive Error Interrupt */
    if (intrs & _BV(RXERIF)) {
        if (enc28j60_get_rx_free_space() <= 0) {
            printf_P(PSTR("    RX overflow\n"));
            // drop packet or process?
        }
        bfc(ENC28J60_EIR, RXERIF);
    }

    /* Transmit Error Interrupt */
    if (intrs & _BV(TXERIF)) {
        
    }

    /* TX Interrupt */
    if (intrs & _BV(TXIF)) {
        
    }

    /* Link Change Interrupt */
    if (intrs & _BV(LINKIF)) {
        printf_P(PSTR("    Link is %S\n"), check_link() ? PSTR("Up") : PSTR("Down"));

        /* read PHIR to clear LINKIF */
        phy_read(ENC28J60_PHIR);
    }

    /* DMA Interrupt */
    if (intrs & _BV(DMAIF)) {
        bfc(ENC28J60_EIR, _BV(DMAIF));
    }

    /* Receive Packet Pending Interrupt
     * PKTIF is unreliable (Errate #6)
     * check EPKTCNT
     */
    if (intrs & _BV(PKTIF)) {
        printf_P(PSTR("    Packet receive\n"));
        enc28j60_packet_receive();
    }

    bfs(ENC28J60_EIE, _BV(INTIE));  // enable interrupts
}
