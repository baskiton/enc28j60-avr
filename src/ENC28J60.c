#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <defines.h>
#include <spi.h>
#include <net/net.h>
#include <net/net_dev.h>
#include <net/socket.h>

#include "ENC28J60.h"

struct enc28j60_dev {
    struct net_dev_s *net_dev;
    struct spi_device_s *spi_dev;
    uint16_t packet_ptr;
    struct net_buff_s *tx_nbuff;
};

static uint8_t rcr(const struct enc28j60_dev *priv, uint8_t addr);
static void wcr(const struct enc28j60_dev *priv, uint8_t addr, uint8_t data);


/*!
 * @brief Change the memory bank
 * @param addr Register address
 */
static void change_memory_bank(const struct enc28j60_dev *priv, uint8_t addr) {
    // return if it is Common Register
    if ((addr >= ENC28J60_EIE) && (addr <= ENC28J60_ECON1))
        return;

    uint8_t bank = (addr & ENC28J60_BANK_MASK) >> 5;
    
    uint8_t econ1;

    econ1 = rcr(priv, ENC28J60_ECON1);
    econ1 = (econ1 & 0b11111100U) | bank;
    wcr(priv, ENC28J60_ECON1, econ1);
}

/*!
 * @brief Read Control Register
 * @param addr Control Register Address
 * @param dummy If the address specifies one of the MAC or MII registers,
 * a dummy byte will first be shifted out on the SO pin.
 * @return 8-bit data from register
 */
static uint8_t rcr(const struct enc28j60_dev *priv, uint8_t addr) {
    uint8_t result;

    change_memory_bank(priv, addr);
    
    chip_select(priv->spi_dev->cs);
    spi_write(ENC28J60_RCR_OP | (addr & ENC28J60_ADDR_MASK));
    if (addr & ENC28J60_DUMMY_MASK)  // dummy read
        spi_read_8();
    result = spi_read_8();
    chip_desel(priv->spi_dev->cs);

    return result;
}

/*!
 * @brief Read Buffer Memory
 * @param buf Buffer to write receiving data
 * @param count Number of bytes to read
 * @param addr Address of start reading
 */
static void rbm(const struct enc28j60_dev *priv, void *buf,
                uint16_t count, uint16_t addr) {
    wcr(priv, ENC28J60_ERDPTL, (uint8_t)addr);
    wcr(priv, ENC28J60_ERDPTH, (uint8_t)(addr >> 8));

    chip_select(priv->spi_dev->cs);
    spi_write(ENC28J60_RBM_OP);
    spi_read_buf((uint8_t *)buf, count);
    chip_desel(priv->spi_dev->cs);
}

/*!
 * @brief Write Control Register
 * @param addr Control Register Address
 * @param data 8-bit write data
 */
static void wcr(const struct enc28j60_dev *priv, uint8_t addr, uint8_t data) {
    change_memory_bank(priv, addr);

    chip_select(priv->spi_dev->cs);
    spi_write(ENC28J60_WCR_OP | (addr & ENC28J60_ADDR_MASK));
    spi_write(data);
    chip_desel(priv->spi_dev->cs);
}

/*!
 * @brief Write Buffer Memory
 * @param buf Data sequence to send
 * @param count Number of bytes in \p buf to write
 */
static void wbm(const struct enc28j60_dev *priv,
                const void *buf, uint16_t count) {
    chip_select(priv->spi_dev->cs);
    spi_write(ENC28J60_WBM_OP);
    spi_write_buf((uint8_t *)buf, count);
    chip_desel(priv->spi_dev->cs);
}

/*!
 * @brief Bit Field Set. ONLY FOR ETH CONTROL REGISTERS!!!
 * @param addr Control Register Address
 * @param bf Bit Field to set
 */
static void bfs(const struct enc28j60_dev *priv, uint8_t addr, uint8_t bf) {
    change_memory_bank(priv, addr);

    chip_select(priv->spi_dev->cs);
    spi_write(ENC28J60_BFS_OP | (addr & ENC28J60_ADDR_MASK));
    spi_write(bf);
    chip_desel(priv->spi_dev->cs);
}

/*!
 * @brief Bit Field Clear. ONLY FOR ETH CONTROL REGISTERS!!!
 * @param addr 5-bit Control Register Address
 * @param bf Bit Field to clear
 */
static void bfc(const struct enc28j60_dev *priv, uint8_t addr, uint8_t bf) {
    change_memory_bank(priv, addr);

    chip_select(priv->spi_dev->cs);
    spi_write(ENC28J60_BFC_OP | (addr & ENC28J60_ADDR_MASK));
    spi_write(bf);
    chip_desel(priv->spi_dev->cs);
}

/*!
 * @brief Reading PHY register
 * @param reg PHY Register Address
 * @return 16-bit data from \p reg
 */
static uint16_t phy_read(const struct enc28j60_dev *priv, uint8_t reg) {
    uint16_t result = 0;

    wcr(priv, ENC28J60_MIREGADR, reg);
    bfs(priv, ENC28J60_MICMD, _BV(MIIRD));    // starting read
    _delay_us(10.24);
    while (rcr(priv, ENC28J60_MISTAT) & _BV(BUSY)) {}
    bfc(priv, ENC28J60_MICMD, _BV(MIIRD));
    result = rcr(priv, ENC28J60_MIRDH) << 8;
    result |= rcr(priv, ENC28J60_MIRDL);

    return result;
}

/*!
 * @brief Writing PHY register.
 * @param reg PHY Register Address
 * @param data 16-bit write data
 */
static void phy_write(const struct enc28j60_dev *priv,
                      uint8_t reg, uint16_t data) {
    wcr(priv, ENC28J60_MIREGADR, reg);
    wcr(priv, ENC28J60_MIWRL, (uint8_t)(data & 0xFF));
    wcr(priv, ENC28J60_MIWRH, (uint8_t)(data >> 8));  // starting write
    _delay_us(10.24);
    while (rcr(priv, ENC28J60_MISTAT) & _BV(BUSY)) {}
}

/*!
 * @brief Automatic scanning specific PHY register.
 * Data will be written to the \p MIRD registers every 10.24 μs.
 * When the \p MIISCAN operation is in progress, the host controller
 * must not attempt to write to \p MIWRH or start an \p MIIRD operation.
 * @param reg PHY Register Address
 */
static void phy_scan_start(const struct enc28j60_dev *priv, uint8_t reg) {
    wcr(priv, ENC28J60_MIREGADR, reg);
    bfs(priv, ENC28J60_MICMD, _BV(MIISCAN));  // starting scan
}

/*!
 * @brief Stop automatically scanning of PHY register
 */
static void phy_scan_stop(const struct enc28j60_dev *priv) {
    bfc(priv, ENC28J60_MICMD, _BV(MIISCAN));
    while (rcr(priv, ENC28J60_MISTAT) & _BV(BUSY)) {}
}

/*!
 * @brief Get value with autoscan PHY-gerister.
 * Since the host controller can only read one MII register at a time,
 * the \p MIRDL and \p MIRDH values ​​will be read from the PHY at different times.
 * @return 16-bit data from PHY-register
 */
static uint16_t phy_scan_rd(const struct enc28j60_dev *priv) {
    uint16_t result = 0;

    while (rcr(priv, ENC28J60_MISTAT) & _BV(NVALID)) {}

    result = rcr(priv, ENC28J60_MIRDH) << 8;
    result |= rcr(priv, ENC28J60_MIRDL);

    return result;
}

/*!
 * @brief Get OUI from PHY-registers and making it canonical
 * @param oui 3-bytes OUI buffer to store
 */
static void get_oui(const struct enc28j60_dev *priv, uint8_t *oui) {
    uint32_t tmp;

    tmp = phy_read(priv, ENC28J60_PHID2) >> 10;
    tmp |= ((uint32_t)phy_read(priv, ENC28J60_PHID1) << 6);

    oui[0] = bits_swap(((tmp >> 16) & 0xFFU));
    oui[1] = bits_swap(((tmp >> 8) & 0xFFU));
    oui[2] = bits_swap((tmp & 0xFFU));
}

/*!
 * ERXRDPT need to be set always at odd addresses
 * (by ERRATA, issue #14)
 */
static uint16_t erxrdpt_workaround(uint16_t next_packet_ptr,
                                   uint16_t rxst, uint16_t rxnd) {
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
 * @return 0 if success; errno if error
 */
static int8_t rx_buf_init(struct enc28j60_dev *priv,
                          uint16_t start, uint16_t end) {
    uint16_t erxrdpt;

    if ((start > 0x1FFF) || (end > 0x1FFF) || (start > end)) {
        printf_P(PSTR("rx_buf_init(%d, %d) - RX buf bad parameters!\n"), start, end);
        // errno
        return -1;
    }
    
    priv->packet_ptr = start;
    wcr(priv, ENC28J60_ERXSTL, (uint8_t)start);
    wcr(priv, ENC28J60_ERXSTH, (uint8_t)(start >> 8));

    wcr(priv, ENC28J60_ERXNDL, (uint8_t)end);
    wcr(priv, ENC28J60_ERXNDH, (uint8_t)(end >> 8));

    erxrdpt = erxrdpt_workaround(priv->packet_ptr, start, end);
    wcr(priv, ENC28J60_ERXRDPTL, (uint8_t)erxrdpt);
    wcr(priv, ENC28J60_ERXRDPTH, (uint8_t)(erxrdpt >> 8));

    return 0;
}

/*!
 * @brief TX buffer initialize.
 * ETXST and ATXND will not be changed after transmit operation.
 * @param start Start address of TX buffer
 * @param end End address of TX buffer
 * @return 0 if success; errno if error
 */
static int8_t tx_buf_init(struct enc28j60_dev *priv,
                          uint16_t start, uint16_t end) {
    if ((start > 0x1FFF) || (end > 0x1FFF) || (start > end)) {
        printf_P(PSTR("tx_buf_init(%d, %d) - TX buf bad parameters!\n"), start, end);
        // errno
        return -1;
    }
    
    wcr(priv, ENC28J60_ETXSTL, (uint8_t)start);
    wcr(priv, ENC28J60_ETXSTH, (uint8_t)(start >> 8));

    wcr(priv, ENC28J60_ETXNDL, (uint8_t)end);
    wcr(priv, ENC28J60_ETXNDH, (uint8_t)(end >> 8));

    return 0;
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
static void enc28j60_enable(const struct enc28j60_dev *priv) {
    /* enable interrupt logic */

    phy_write(priv, ENC28J60_PHIR, 0); // clear all PHY interrupt flags
    phy_write(priv, ENC28J60_PHIE, (_BV(PGEIE) | _BV(PLNKIE)));

    // clear all interrupt flags
    bfc(priv, ENC28J60_EIR, (_BV(PKTIF) | _BV(DMAIF) | _BV(LINKIF) |
                                      _BV(TXIF) | _BV(TXERIF) | _BV(RXERIF)));
    // enable interrupts (except for DMA, which is currently not used)
    wcr(priv, ENC28J60_EIE, (_BV(INTIE) | _BV(PKTIE) | _BV(LINKIE) |
                                      _BV(TXIE) | _BV(TXERIF) | _BV(RXERIE)));

    /* enable receive */
    bfs(priv, ENC28J60_ECON1, _BV(RXEN));
}

/*!
 * @brief Disable chip interrupts and packet reception
 */
static void enc28j60_disable(struct net_dev_s *net_dev) {
    struct enc28j60_dev *priv = net_dev->priv;

    // disable all interrupts
    phy_write(priv, ENC28J60_PHIE, 0);
    wcr(priv, ENC28J60_EIE, 0);
    
    // disable packet reception
    bfc(priv, ENC28J60_ECON1, _BV(RXEN));

    net_dev_tx_disallow(net_dev);
}

/*!
 * @brief System reset command (soft reset)
 */
static void enc28j60_soft_reset(struct enc28j60_dev *priv) {
    chip_select(priv->spi_dev->cs);
    spi_write(ENC28J60_SRC_OP);
    chip_desel(priv->spi_dev->cs);

    // _delay_us(50);    // delay 50 us after soft reset
    _delay_us(1000);    // delay at least 1 ms. Errata issue #2
}

static int8_t enc28j60_write_mac_addr(struct enc28j60_dev *priv) {
    uint8_t *mac = priv->net_dev->dev_addr;

    if (rcr(priv, ENC28J60_ECON1) & _BV(RXEN)) {
        // device must be disable to set MAC addr
        // EBUSY
        return -1;
    }
    
    wcr(priv, ENC28J60_MAADR1, mac[0]);
    wcr(priv, ENC28J60_MAADR2, mac[1]);
    wcr(priv, ENC28J60_MAADR3, mac[2]);
    wcr(priv, ENC28J60_MAADR4, mac[3]);
    wcr(priv, ENC28J60_MAADR5, mac[4]);
    wcr(priv, ENC28J60_MAADR6, mac[5]);

    return 0;
}

/*!
 * @brief Set the new MAC address. Device must be disable to set MAC addr
 * @param mac Pointer to buffer with new MAC addr (struct sock_addr???)
 * @return 0 if success, errno if error
 */
static int8_t enc28j60_set_mac_addr(struct net_dev_s *net_dev, const void *addr) {
    // struct sock_addr *a = addr;
    if (net_dev_upstate_is_run(net_dev)) {
        // Device must be disable
        // EBUSY
        return -1;
    }

    memcpy(net_dev->dev_addr, addr, ETH_MAC_LEN);
    
    return enc28j60_write_mac_addr(net_dev->priv);
}

/*!
 * @brief Get Ethernet Revision ID
 * @return 5-bit revision ID
 */
uint8_t enc28j60_read_rev_id(const struct net_dev_s *net_dev) {
    uint8_t result;
    struct enc28j60_dev *priv = net_dev->priv;

    result = rcr(priv, ENC28J60_EREVID) & 0x1F;

    return result;
}

/*!
 * @brief Get MAC-address
 * @param mac_buf Buffer to store MAC-addr.
 * Make sure that buffer is long enough (default 6 bytes).
 */
static void enc28j60_get_mac(const struct enc28j60_dev *priv, uint8_t *mac_buf) {
    mac_buf[0] = rcr(priv, ENC28J60_MAADR1);
    mac_buf[1] = rcr(priv, ENC28J60_MAADR2);
    mac_buf[2] = rcr(priv, ENC28J60_MAADR3);
    mac_buf[3] = rcr(priv, ENC28J60_MAADR4);
    mac_buf[4] = rcr(priv, ENC28J60_MAADR5);
    mac_buf[5] = rcr(priv, ENC28J60_MAADR6);
}

/*!
 * @brief Transmitting Packet
 * @param net_buff Pointer to buffer with data
 * @param dev Device
 * @return \a NETDEV_TX_OK if transfer ok;
 *         \a NETDEV_TX_BUSY if tx was busy
 */
static int8_t enc28j60_packet_transmit(struct net_buff_s *net_buff,
                                     struct net_dev_s *net_dev) {
    /* Per Packet Control Byte. 0 by default.
        Otherwise, refer to the datasheet on chapter 7.1 */
    uint8_t ppcb = 0;
    uint16_t end;
    struct enc28j60_dev *priv = net_dev->priv;

    /* check if transfer is in progress. just in case */
    if (rcr(priv, ENC28J60_ECON1) & _BV(TXRTS)) {
        return NETDEV_TX_BUSY;
    }

    net_dev_tx_disallow(net_dev);

    priv->tx_nbuff = net_buff;
    
    /* set EWRPT - pointer to start of transmit buffer */
    wcr(priv, ENC28J60_EWRPTL, (uint8_t)ENC28J60_TXSTART_INIT);
    wcr(priv, ENC28J60_EWRPTH, (uint8_t)(ENC28J60_TXSTART_INIT >> 8));
    
    /* write to buffer */
    wbm(priv, &ppcb, 1);
    wbm(priv, net_buff->head, net_buff->pkt_len);

    /* set ETXND. It should point to the last byte in the data payload. */
    end = ENC28J60_TXSTART_INIT + net_buff->pkt_len;
    wcr(priv, ENC28J60_ETXNDL, (uint8_t)end);
    wcr(priv, ENC28J60_ETXNDH, (uint8_t)(end >> 8));

    /* Clear EIR.TXIF. Needed? */
    bfc(priv, ENC28J60_EIR, _BV(TXIF));

    /* Start the transmission process */
    bfs(priv, ENC28J60_ECON1, _BV(TXRTS));

    free_net_buff(priv->tx_nbuff);
    priv->tx_nbuff = NULL;

    return NETDEV_TX_OK;
}

/*!
 * @brief Config a receive filter
 */
void enc28j60_set_rx_mode(struct net_dev_s *net_dev) {
    struct enc28j60_dev *priv = net_dev->priv;
    uint8_t mode = 0;

    if (net_dev->flags.rx_mode & RX_RT_PROMISC) {
        mode = 0x00;
    } else if (net_dev->flags.rx_mode & RX_RT_ALLMULTI) {
        mode = _BV(UCEN) | _BV(CRCEN) | _BV(MCEN) | _BV(BCEN);
    // } else if (net_dev->flags.rx_mode & RX_RT_UNICAST) {
    //     mode = _BV(UCEN) | _BV(CRCEN);
    // } else if (net_dev->flags.rx_mode & RX_RT_MULTICAST) {
    //     mode = _BV(UCEN) | _BV(CRCEN) | _BV(MCEN);
    } else {    // RX_RT_BROADCAST - 0 by default
        mode = _BV(UCEN) | _BV(CRCEN) | _BV(BCEN);
    }

    wcr(priv, ENC28J60_ERXFCON, mode);
}

/*!
 * @brief Receiving Packet
 */
static void enc28j60_packet_receive(struct enc28j60_dev *priv) {
    rsv_t rsv;
    uint16_t erxrdpt;
    struct net_buff_s *data;
    struct net_dev_s *net_dev = priv->net_dev;

    if (priv->packet_ptr > ENC28J60_RXEND_INIT) {
        // packet address corrupted
        bfs(priv, ENC28J60_ECON1, RXRST);
        bfc(priv, ENC28J60_ECON1, RXRST);
        if (rx_buf_init(priv, ENC28J60_RXSTART_INIT, ENC28J60_RXEND_INIT))
            return;
        bfc(priv, ENC28J60_EIR, RXERIF);    // clr this cause receive was aborted
        bfs(priv, ENC28J60_ECON1, RXEN);    // rx enable and...
        return; // and exit without decrement packets count
    }

    /* reading receive status vector */
    rbm(priv, (uint8_t *)&rsv, ENC28J60_RSV_SIZE, priv->packet_ptr);
    
    if (!(rsv.rsv_rx_ok) || (rsv.rx_byte_cnt > ENC28J60_MAX_FRAME_LEN)) {
        printf_P(PSTR("--- Receive Packet Error!  ---\n"));
        if (rsv.rsv_crc_err)
            printf_P(PSTR("--- --- CRC Error! --- --- ---\n"));
        if (rsv.rsv_len_check_err)
            printf_P(PSTR("--- --- Lenght Check Error! --\n"));
        if (rsv.rx_byte_cnt > ENC28J60_MAX_FRAME_LEN)
            printf_P(PSTR("--- --- Max Lenght Error - %u\n"), rsv.rx_byte_cnt);
        rsv.next_packet_ptr = priv->packet_ptr;
    } else {
        /* reading received packets */
        data = ndev_alloc_net_buff(net_dev, rsv.rx_byte_cnt);
        if (!data) {
            // what to do if not enough memory?
            printf_P(PSTR("Out of memory for RX buffer. Need %d byte(s). Avail %ld byte(s)\n"),
                     rsv.rx_byte_cnt, get_free_mem_size());
        } else {
            rbm(priv, put_net_buff(data, rsv.rx_byte_cnt), rsv.rx_byte_cnt,
                rand_acc_addr_calc(priv->packet_ptr, ENC28J60_RSV_SIZE));
            data->protocol = eth_type_proto(data, net_dev);
            data->flags.ip_summed = CHECKSUM_HW;
            /**
             * TODO:
             *  Here it is necessary to call the packet handler or somehow
             *  notify the controller about the start of processing.
             * ATTENTION:
             *  IN THE HANDLER, YOU MUST REMEMBER TO FREE THE ALLOCATED MEMORY.
             */
        }
    }

    /* freeing receive buffer space */
    erxrdpt = erxrdpt_workaround(rsv.next_packet_ptr, ENC28J60_RXSTART_INIT,
                                 ENC28J60_RXEND_INIT);
    wcr(priv, ENC28J60_ERXRDPTL, (uint8_t)erxrdpt);
    wcr(priv, ENC28J60_ERXRDPTH, (uint8_t)(erxrdpt >> 8));

    priv->packet_ptr = rsv.next_packet_ptr;
    bfs(priv, ENC28J60_ECON2, _BV(PKTDEC));   // decrement packet count
}

/*!
 * @brief Get size of free space of RX buffer
 * @return Free space in the RX buffer
 */
static int16_t enc28j60_get_rx_free_space(const struct enc28j60_dev *priv) {
    uint8_t epktcnt;
    uint16_t erxwrpt, erxrdpt, erxst, erxnd;
    int16_t result;

    epktcnt = rcr(priv, ENC28J60_EPKTCNT);
    if (epktcnt == 255)
        return -1;
    
    while (true) {
        erxwrpt = (uint16_t)rcr(priv, ENC28J60_ERXWRPTL);
        erxwrpt |= (uint16_t)rcr(priv, ENC28J60_ERXWRPTH) << 8;
        if (epktcnt == rcr(priv, ENC28J60_EPKTCNT))
            break;
    }

    erxrdpt = (uint16_t)rcr(priv, ENC28J60_ERXRDPTL);
    erxrdpt |= (uint16_t)rcr(priv, ENC28J60_ERXRDPTH) << 8;

    erxst = (uint16_t)rcr(priv, ENC28J60_ERXSTL);
    erxst |= (uint16_t)rcr(priv, ENC28J60_ERXSTH) << 8;

    erxnd = (uint16_t)rcr(priv, ENC28J60_ERXNDL);
    erxnd |= (uint16_t)rcr(priv, ENC28J60_ERXNDH) << 8;

    if (erxwrpt > erxrdpt)
        result = (erxnd - erxst) - (erxwrpt - erxrdpt);
    else if (erxwrpt == erxrdpt)
        result = erxnd - erxst;
    else
        result = erxrdpt - erxwrpt - 1;

    return result;
}

/*!
 * @brief Check Link Status and set flag
 * @return True if Link Up; False otherwise
 */
static bool enc28j60_check_link(const struct enc28j60_dev *priv) {
    struct net_dev_s *net_dev = priv->net_dev;

    if (phy_read(priv, ENC28J60_PHSTAT2) & _BV(LSTAT)) {
        net_dev_set_link_up(net_dev);
    } else {
        net_dev_set_link_down(net_dev);
    }

    return net_dev_link_is_up(net_dev);
}

/*!
 * @brief Interrupt Request Handler
 */
void enc28j60_irq_handler(struct net_dev_s *net_dev) {
    uint8_t intrs;
    tsv_t tsv;
    struct enc28j60_dev *priv = net_dev->priv;

    bfc(priv, ENC28J60_EIE, _BV(INTIE));  // disable interrupts

    intrs = rcr(priv, ENC28J60_EIR);

    /* Receive Error Interrupt
     * RX buffer overflow condition or too many packets are in the RX buffer
     * and more cannot be stored without overflowing the EPKTCNT register
     */
    if (intrs & _BV(RXERIF)) {
        if (enc28j60_get_rx_free_space(priv) <= 0) {
            printf_P(PSTR("    RX overflow\n"));
            // pass or process?
        }
        /* Flow Control - Send Pause Control Frame
        (by the IEEE 802.3 specification).
        The packet being received will be aborted (permanently lost) */
        if (net_dev->flags.full_duplex) {
            // send one pause frame then turn flow control off
            wcr(priv, ENC28J60_EFLOCON, _BV(FCEN0));
        }
        /* If half-duplex, flow control will automatically sending
        pause frame until host-controller tells to transmit a packet.
        This can cause collisions, so half-duplex flow control
        is not recommended. */

        bfc(priv, ENC28J60_EIR, _BV(RXERIF));
    }

    /* TX Interrupt */
    if ((intrs & _BV(TXIF)) && !(intrs & _BV(TXERIF))) {
        if (rcr(priv, ENC28J60_ESTAT) & _BV(TXABRT)) {
            // abort
            printf_P(PSTR("    TX abort\n"));
        } else {
            // transmit success
        }
        net_dev_tx_allow(net_dev);

        bfc(priv, ENC28J60_EIR, _BV(TXIF));
    }

    /* Transmit Error Interrupt */
    if (intrs & _BV(TXERIF)) {
        rbm(priv, (uint8_t *)&tsv, ENC28J60_TSV_SIZE,
            (((uint16_t)rcr(priv, ENC28J60_ETXNDH) << 8) |
             (uint16_t)(rcr(priv, ENC28J60_ETXNDL) + 1)));
        
        /* reset tx logic */
        bfs(priv, ENC28J60_ECON1, TXRST);
        bfc(priv, ENC28J60_ECON1, TXRST);
        tx_buf_init(priv, ENC28J60_TXSTART_INIT, ENC28J60_TXEND_INIT);
        
        /** check tsv to search a problem
         *  TODO: for half-duplex:
         *      if collision occured, check for retransmit
         */

        printf_P(PSTR("    TX error\n"));

        net_dev_tx_allow(net_dev);

        bfc(priv, ENC28J60_EIR, (_BV(TXIF) | _BV(TXERIF)));
    }

    /* Link Change Interrupt */
    if (intrs & _BV(LINKIF)) {
        enc28j60_check_link(priv);
        /* read PHIR to clear LINKIF, PGIF and PLNKIF */
        phy_read(priv, ENC28J60_PHIR);
    }

    /* DMA Interrupt.
     * currently not used (DMAIE = 0)
     */
    if (intrs & _BV(DMAIF)) {
        /* DMA module has completed its memory copy or checksum calculation
         * or the host controller cancels a DMA operation by
         * manually clearing the DMAST bit
         */
        bfc(priv, ENC28J60_EIR, _BV(DMAIF));
    }

    /* Receive Packet Pending Interrupt
     * PKTIF is unreliable (Errate #6)
     * check EPKTCNT
     */
    if (rcr(priv, ENC28J60_EPKTCNT)) {
        enc28j60_packet_receive(priv);
    }

    bfs(priv, ENC28J60_EIE, _BV(INTIE));  // enable interrupts
}

/*!
 * @brief Initial ethernet shield sequence
 * @param priv Private data of this device
 * @return 0 if success; errno if error
 */
static int8_t enc28j60_init(struct enc28j60_dev *priv) {
    uint8_t revid, sreg, err;
    struct net_dev_s *net_dev = priv->net_dev;

    sreg = SREG;
    cli();

    chip_desel(priv->spi_dev->cs);
    
    // spi_set_speed(ENC28J60_MAX_FREQ);
    
    /* Oscillator Start-up Timer (OST) - 300 us delay after power-up */
    _delay_us(300);

    /* reset routine */
    enc28j60_soft_reset(priv);

    net_dev_set_upstate_stop(net_dev);

    /* INITIALIZATION */
    revid = rcr(priv, ENC28J60_EREVID) & 0x1F;
    if ((revid == 0x00) || (revid == 0xFFU)) {
        // device not connect
        printf_P(PSTR("enc28j60_init() - Invalid REVID %d\n"), revid);
        return -1;
    }
    
    /* Receive Buffer init*/
    err = rx_buf_init(priv, ENC28J60_RXSTART_INIT, ENC28J60_RXEND_INIT);
    if (err)
        return err;
    
    /* Transmit Buffer init */
    err = tx_buf_init(priv, ENC28J60_TXSTART_INIT, ENC28J60_TXEND_INIT);
    if (err)
        return err;

    /* Receive Filters
       The appropriate receive filters should be enabled or
       disabled by writing to the ERXFCON register.
       By default is set UCEN, CRCEN, BCEN
       (unicast, broadcast and CRC check)
     */
    wcr(priv, ENC28J60_ERXFCON, (_BV(UCEN) | _BV(CRCEN) | _BV(BCEN)));

    /* Waiting for OST
       CLKRDY bit must be polled before
       transmitting packets, enabling packet
       reception or accessing any MAC, MII or
       PHY registers.
     */
    while (!(rcr(priv, ENC28J60_ESTAT) & _BV(CLKRDY))) {}

    enc28j60_disable(net_dev);
    
    /* MAC Initialization Settings
        1. Set the MARXEN bit in MACON1 to enable the MAC to receive frames.
            If using full duplex, also set TXPAUS and RXPAUS.
        2. Configure the PADCFG (101), TXCRCEN and FULDPX bits of MACON3.
            Set the FRMLNEN bit to enable frame length status reporting.
            The FULDPX bit should be set if the app will be connected to a
            full-duplex configured remote node; otherwise, it should be left clear.
        3. Configure the bits in MACON4. For conformance to
            the IEEE 802.3 standard, set the DEFER bit.
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
        9. Program the local MAC address into the MAADR1:MAADR6 registers
            (after initialization).
     */
    if (net_dev->flags.full_duplex) {
        wcr(priv, ENC28J60_MACON1, (_BV(MARXEN) | _BV(TXPAUS) | _BV(RXPAUS)));  // 1.
        wcr(priv, ENC28J60_MACON3, (_BV(PADCFG2) | _BV(PADCFG0) | _BV(TXCRCEN) |
                                    _BV(FRMLNEN) | _BV(FULDPX)));   // 2.
        wcr(priv, ENC28J60_MACON4, _BV(DEFER));   // 3.
        wcr(priv, ENC28J60_MABBIPG, 0x15);   // 5.
    } else {
        wcr(priv, ENC28J60_MACON1, _BV(MARXEN));  // 1.
        wcr(priv, ENC28J60_MACON3, (_BV(PADCFG2) | _BV(PADCFG0) |
                                    _BV(TXCRCEN) | _BV(FRMLNEN)));  // 2.
        wcr(priv, ENC28J60_MACON4, (_BV(DEFER) | _BV(BPEN) | _BV(NOBKOFF)));    // 3.
        wcr(priv, ENC28J60_MABBIPG, 0x12);   // 5.
        wcr(priv, ENC28J60_MAIPGH, 0x0C);    // 7.
        wcr(priv, ENC28J60_MACLCON1, 0x0F);  // 8.   defaul value
        wcr(priv, ENC28J60_MACLCON2, 0x37);  // 8.   defaul value
    }

    wcr(priv, ENC28J60_MAMXFLL, (uint8_t)(ENC28J60_MAX_FRAME_LEN & 0xFFU)); // 4.
    wcr(priv, ENC28J60_MAMXFLH, (uint8_t)(ENC28J60_MAX_FRAME_LEN >> 8));    // MAMXFL = 1518
    wcr(priv, ENC28J60_MAIPGL, 0x12);    // 6.

    /* PHY Initialization Settings
        For proper duplex operation, the PHCON1.PDPXMD bit must
        also match the value of the MACON3.FULDPX bit.
        If using Half-Duplex, may to set the PHCON2.HDLDIS bit to
        prevent automatic loopback of the data which is transmitted.

        The PHLCON register controls the outputs of LEDA and LEDB.
        If an application requires a LED configuration other than the default,
        PHLCON must be altered to match the new requirements.
     */
    if (net_dev->flags.full_duplex) {
        phy_write(priv, ENC28J60_PHCON1,
                  (phy_read(priv, ENC28J60_PHCON1) | _BV(PDPXMD)));
    } else {
        phy_write(priv, ENC28J60_PHCON1,
                  (phy_read(priv, ENC28J60_PHCON1) & ~_BV(PDPXMD)));
        phy_write(priv, ENC28J60_PHCON2,
                  (phy_read(priv, ENC28J60_PHCON2) | _BV(HDLDIS)));
    }

    printf_P(PSTR("ENC28J60 initialized with RevID %d\n"), revid);

    SREG = sreg;

    return 0;
}
/*!
 * @brief Puts this device into working state.
 * Also, the duplex settings will only take effect after enc28j60_init().
 * @return 0 if success; errno if error
 */
static int8_t enc28j60_open(struct net_dev_s *net_dev) {
    struct enc28j60_dev *priv = net_dev->priv;

    enc28j60_disable(net_dev);
    if (enc28j60_init(priv)) {
        // errno
        return -1;
    }
    enc28j60_write_mac_addr(priv);
    enc28j60_enable(priv);
    enc28j60_check_link(priv);
    net_dev_tx_allow(net_dev);

    return 0;
}

/** TODO: move this to PROGMEM */
static const struct net_dev_ops_s enc28j60_net_dev_ops = {
    .init = NULL,
    .open = enc28j60_open,
    .stop = enc28j60_disable,
    .start_tx = enc28j60_packet_transmit,
    .set_rx_mode = enc28j60_set_rx_mode,
    .set_mac_addr = enc28j60_set_mac_addr,
    .irq_handler = enc28j60_irq_handler,
};

/*!
 * @brief
 * @param spi_dev
 * @return 0 if success; errno if error
 */
int8_t enc28j60_probe(spi_dev_t *spi_dev) {
    struct net_dev_s *ndev;
    struct enc28j60_dev *priv;
    int8_t ret;
    uint8_t *mac;

    ndev = eth_dev_alloc(sizeof(struct enc28j60_dev));
    if (!ndev) {
        // ENOMEM
        return -1;
    }
    priv = ndev->priv;
    spi_dev->priv_data = priv;
    priv->net_dev = ndev;
    priv->spi_dev = spi_dev;

    enc28j60_get_mac(priv, mac);

    ret = enc28j60_init(priv);
    if (ret) {
        // EIO
        net_dev_free(ndev);
        return ret;
    }

    mac = ndev->dev_addr;
    get_oui(priv, mac);     // get manufacture MAC with OUI and write to ndev->dev_addr
    mac[3] = mac[4] = mac[5] = 0;
    enc28j60_write_mac_addr(priv);

    /** TODO: here need to configure the interrupt handler */

    ndev->netdev_ops = &enc28j60_net_dev_ops;

    ret = netdev_register(ndev);
    if (ret) {
        // register error
        /** TODO: here need to free the interrupt handler */
        net_dev_free(ndev);
        return ret;
    }

    return ret;
}

/*!
 * @brief Remove this device and free up the memory it occupies.
 * @param spi_dev Pointer to the SPI-structure associated with the device.
 */
void enc28j60_remove(spi_dev_t *spi_dev) {
    struct enc28j60_dev *priv = spi_dev->priv_data;

    netdev_unregister(priv->net_dev);
    /** TODO: here need to free the interrupt handler */
    net_dev_free(priv->net_dev);
}
