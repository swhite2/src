/*
 * AMD 10Gb Ethernet driver
 *
 * This file is available to you under your choice of the following two
 * licenses:
 *
 * License 1: GPLv2
 *
 * Copyright (c) 2014-2016 Advanced Micro Devices, Inc.
 *
 * This file is free software; you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or (at
 * your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *     The Synopsys DWC ETHER XGMAC Software Driver and documentation
 *     (hereinafter "Software") is an unsupported proprietary work of Synopsys,
 *     Inc. unless otherwise expressly agreed to in writing between Synopsys
 *     and you.
 *
 *     The Software IS NOT an item of Licensed Software or Licensed Product
 *     under any End User Software License Agreement or Agreement for Licensed
 *     Product with Synopsys or any supplement thereto.  Permission is hereby
 *     granted, free of charge, to any person obtaining a copy of this software
 *     annotated with this license and the Software, to deal in the Software
 *     without restriction, including without limitation the rights to use,
 *     copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *     of the Software, and to permit persons to whom the Software is furnished
 *     to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included
 *     in all copies or substantial portions of the Software.
 *
 *     THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 *     BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *     TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *     PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS
 *     BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *     INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *     ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *     THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * License 2: Modified BSD
 *
 * Copyright (c) 2014-2016 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *     The Synopsys DWC ETHER XGMAC Software Driver and documentation
 *     (hereinafter "Software") is an unsupported proprietary work of Synopsys,
 *     Inc. unless otherwise expressly agreed to in writing between Synopsys
 *     and you.
 *
 *     The Software IS NOT an item of Licensed Software or Licensed Product
 *     under any End User Software License Agreement or Agreement for Licensed
 *     Product with Synopsys or any supplement thereto.  Permission is hereby
 *     granted, free of charge, to any person obtaining a copy of this software
 *     annotated with this license and the Software, to deal in the Software
 *     without restriction, including without limitation the rights to use,
 *     copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *     of the Software, and to permit persons to whom the Software is furnished
 *     to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included
 *     in all copies or substantial portions of the Software.
 *
 *     THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 *     BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *     TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *     PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS
 *     BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *     INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *     ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *     THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "xgbe.h"
#include "xgbe-common.h"

extern unsigned int g_axgbe_debug_level;

int
xgbe_calc_rx_buf_size(struct ifnet *netdev, unsigned int mtu)
{
	unsigned int rx_buf_size;

	if (mtu > XGMAC_JUMBO_PACKET_MTU)
		return (-EINVAL);

	rx_buf_size = mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
	rx_buf_size = min(max(rx_buf_size, XGBE_RX_MIN_BUF_SIZE), PAGE_SIZE);
	rx_buf_size = (rx_buf_size + XGBE_RX_BUF_ALIGN - 1) & 
	    ~(XGBE_RX_BUF_ALIGN - 1);

	return (rx_buf_size);
}

void
xgbe_get_all_hw_features(struct xgbe_prv_data *pdata)
{
	unsigned int mac_hfr0, mac_hfr1, mac_hfr2;
	struct xgbe_hw_features *hw_feat = &pdata->hw_feat;

	DBGPR("-->xgbe_get_all_hw_features\n");

	mac_hfr0 = XGMAC_IOREAD(pdata, MAC_HWF0R);
	mac_hfr1 = XGMAC_IOREAD(pdata, MAC_HWF1R);
	mac_hfr2 = XGMAC_IOREAD(pdata, MAC_HWF2R);

	memset(hw_feat, 0, sizeof(*hw_feat));

	hw_feat->version = XGMAC_IOREAD(pdata, MAC_VR);

	/* Hardware feature register 0 */
	hw_feat->gmii        = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, GMIISEL);
	hw_feat->vlhash      = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, VLHASH);
	hw_feat->sma         = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, SMASEL);
	hw_feat->rwk         = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, RWKSEL);
	hw_feat->mgk         = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, MGKSEL);
	hw_feat->mmc         = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, MMCSEL);
	hw_feat->aoe         = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, ARPOFFSEL);
	hw_feat->ts          = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, TSSEL);
	hw_feat->eee         = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, EEESEL);
	hw_feat->tx_coe      = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, TXCOESEL);
	hw_feat->rx_coe      = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, RXCOESEL);
	hw_feat->addn_mac    = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R,
					      ADDMACADRSEL);
	hw_feat->ts_src      = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, TSSTSSEL);
	hw_feat->sa_vlan_ins = XGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, SAVLANINS);

	/* Hardware feature register 1 */
	hw_feat->rx_fifo_size  = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R,
						RXFIFOSIZE);
	hw_feat->tx_fifo_size  = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R,
						TXFIFOSIZE);
	hw_feat->adv_ts_hi     = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, ADVTHWORD);
	hw_feat->dma_width     = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, ADDR64);
	hw_feat->dcb           = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, DCBEN);
	hw_feat->sph           = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, SPHEN);
	hw_feat->tso           = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, TSOEN);
	hw_feat->dma_debug     = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, DBGMEMA);
	hw_feat->rss           = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, RSSEN);
	hw_feat->tc_cnt	       = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, NUMTC);
	hw_feat->hash_table_size = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R,
						  HASHTBLSZ);
	hw_feat->l3l4_filter_num = XGMAC_GET_BITS(mac_hfr1, MAC_HWF1R,
						  L3L4FNUM);

	/* Hardware feature register 2 */
	hw_feat->rx_q_cnt     = XGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, RXQCNT);
	hw_feat->tx_q_cnt     = XGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, TXQCNT);
	hw_feat->rx_ch_cnt    = XGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, RXCHCNT);
	hw_feat->tx_ch_cnt    = XGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, TXCHCNT);
	hw_feat->pps_out_num  = XGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, PPSOUTNUM);
	hw_feat->aux_snap_num = XGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, AUXSNAPNUM);

	/* Translate the Hash Table size into actual number */
	switch (hw_feat->hash_table_size) {
	case 0:
		break;
	case 1:
		hw_feat->hash_table_size = 64;
		break;
	case 2:
		hw_feat->hash_table_size = 128;
		break;
	case 3:
		hw_feat->hash_table_size = 256;
		break;
	}

	/* Translate the address width setting into actual number */
	switch (hw_feat->dma_width) {
	case 0:
		hw_feat->dma_width = 32;
		break;
	case 1:
		hw_feat->dma_width = 40;
		break;
	case 2:
		hw_feat->dma_width = 48;
		break;
	default:
		hw_feat->dma_width = 32;
	}

	/* The Queue, Channel and TC counts are zero based so increment them
	 * to get the actual number
	 */
	hw_feat->rx_q_cnt++;
	hw_feat->tx_q_cnt++;
	hw_feat->rx_ch_cnt++;
	hw_feat->tx_ch_cnt++;
	hw_feat->tc_cnt++;

	/* Translate the fifo sizes into actual numbers */
	hw_feat->rx_fifo_size = 1 << (hw_feat->rx_fifo_size + 7);
	hw_feat->tx_fifo_size = 1 << (hw_feat->tx_fifo_size + 7);
	axgbe_printf(1, "%s: Tx fifo 0x%x Rx fifo 0x%x\n", __func__,
	    hw_feat->tx_fifo_size, hw_feat->rx_fifo_size);

	DBGPR("Hardware features:\n");

	/* Hardware feature register 0 */
	DBGPR("  1GbE support		   : %s\n",
	    hw_feat->gmii ? "yes" : "no");
	DBGPR("  VLAN hash filter	   : %s\n",
	    hw_feat->vlhash ? "yes" : "no");
	DBGPR("  MDIO interface		   : %s\n",
	    hw_feat->sma ? "yes" : "no");
	DBGPR("  Wake-up packet support    : %s\n",
	    hw_feat->rwk ? "yes" : "no");
	DBGPR("  Magic packet support      : %s\n",
	    hw_feat->mgk ? "yes" : "no");
	DBGPR("  Management counters       : %s\n",
	    hw_feat->mmc ? "yes" : "no");
	DBGPR("  ARP offload		   : %s\n",
	    hw_feat->aoe ? "yes" : "no");
	DBGPR("  IEEE 1588-2008 Timestamp  : %s\n",
	    hw_feat->ts ? "yes" : "no");
	DBGPR("  Energy Efficient Ethernet : %s\n",
	    hw_feat->eee ? "yes" : "no");
	DBGPR("  TX checksum offload       : %s\n",
	    hw_feat->tx_coe ? "yes" : "no");
	DBGPR("  RX checksum offload       : %s\n",
	    hw_feat->rx_coe ? "yes" : "no");
	DBGPR("  Additional MAC addresses  : %u\n",
	    hw_feat->addn_mac);
	DBGPR("  Timestamp source	   : %s\n",
	    (hw_feat->ts_src == 1) ? "internal" :
	    (hw_feat->ts_src == 2) ? "external" :
	    (hw_feat->ts_src == 3) ? "internal/external" : "n/a");
	DBGPR("  SA/VLAN insertion	   : %s\n",
	    hw_feat->sa_vlan_ins ? "yes" : "no");

	/* Hardware feature register 1 */
	DBGPR("  RX fifo size		   : %u\n",
	    hw_feat->rx_fifo_size);
	DBGPR("  TX fifo size		   : %u\n",
	    hw_feat->tx_fifo_size);
	DBGPR("  IEEE 1588 high word       : %s\n",
	    hw_feat->adv_ts_hi ? "yes" : "no");
	DBGPR("  DMA width		   : %u\n",
	    hw_feat->dma_width);
	DBGPR("  Data Center Bridging      : %s\n",
	    hw_feat->dcb ? "yes" : "no");
	DBGPR("  Split header		   : %s\n",
	    hw_feat->sph ? "yes" : "no");
	DBGPR("  TCP Segmentation Offload  : %s\n",
	    hw_feat->tso ? "yes" : "no");
	DBGPR("  Debug memory interface    : %s\n",
	    hw_feat->dma_debug ? "yes" : "no");
	DBGPR("  Receive Side Scaling      : %s\n",
	    hw_feat->rss ? "yes" : "no");
	DBGPR("  Traffic Class count       : %u\n",
	    hw_feat->tc_cnt);
	DBGPR("  Hash table size	   : %u\n",
	    hw_feat->hash_table_size);
	DBGPR("  L3/L4 Filters		   : %u\n",
	    hw_feat->l3l4_filter_num);

	/* Hardware feature register 2 */
	DBGPR("  RX queue count		   : %u\n",
	    hw_feat->rx_q_cnt);
	DBGPR("  TX queue count		   : %u\n",
	    hw_feat->tx_q_cnt);
	DBGPR("  RX DMA channel count      : %u\n",
	    hw_feat->rx_ch_cnt);
	DBGPR("  TX DMA channel count      : %u\n",
	    hw_feat->rx_ch_cnt);
	DBGPR("  PPS outputs		   : %u\n",
	    hw_feat->pps_out_num);
	DBGPR("  Auxiliary snapshot inputs : %u\n",
	    hw_feat->aux_snap_num);
	
	DBGPR("<--xgbe_get_all_hw_features\n");
}

void
xgbe_init_tx_coalesce(struct xgbe_prv_data *pdata)
{
	struct xgbe_hw_if *hw_if = &pdata->hw_if;

	DBGPR("-->xgbe_init_tx_coalesce\n");

	pdata->tx_usecs = XGMAC_INIT_DMA_TX_USECS;
	pdata->tx_frames = XGMAC_INIT_DMA_TX_FRAMES;

	hw_if->config_tx_coalesce(pdata);

	DBGPR("<--xgbe_init_tx_coalesce\n");
}

void xgbe_init_rx_coalesce(struct xgbe_prv_data *pdata)
{
	struct xgbe_hw_if *hw_if = &pdata->hw_if;

	DBGPR("-->xgbe_init_rx_coalesce\n");

	pdata->rx_riwt = hw_if->usec_to_riwt(pdata, XGMAC_INIT_DMA_RX_USECS);
	pdata->rx_usecs = XGMAC_INIT_DMA_RX_USECS;
	pdata->rx_frames = XGMAC_INIT_DMA_RX_FRAMES;

	hw_if->config_rx_coalesce(pdata);

	DBGPR("<--xgbe_init_rx_coalesce\n");
}

static void xgbe_free_tx_data(struct xgbe_prv_data *pdata)
{
	struct xgbe_desc_if *desc_if = &pdata->desc_if;
	struct xgbe_channel *channel;
	struct xgbe_ring *ring;
	struct xgbe_ring_data *rdata;
	unsigned int i, j;

	DBGPR("-->xgbe_free_tx_data\n");

	channel = pdata->channel;
	for (i = 0; i < pdata->channel_count; i++, channel++) {
		ring = channel->tx_ring;
		if (!ring)
			break;

		for (j = 0; j < ring->rdesc_count; j++) {
			rdata = XGBE_GET_DESC_DATA(ring, j);
			desc_if->unmap_rdata(pdata, rdata);
		}
	}

	DBGPR("<--xgbe_free_tx_data\n");
}

static void xgbe_free_rx_data(struct xgbe_prv_data *pdata)
{
	struct xgbe_desc_if *desc_if = &pdata->desc_if;
	struct xgbe_channel *channel;
	struct xgbe_ring *ring;
	struct xgbe_ring_data *rdata;
	unsigned int i, j;

	DBGPR("-->xgbe_free_rx_data\n");

	channel = pdata->channel;
	for (i = 0; i < pdata->channel_count; i++, channel++) {
		ring = channel->rx_ring;
		if (!ring)
			break;

		for (j = 0; j < ring->rdesc_count; j++) {
			rdata = XGBE_GET_DESC_DATA(ring, j);
			desc_if->unmap_rdata(pdata, rdata);
		}
	}

	DBGPR("<--xgbe_free_rx_data\n");
}

static int xgbe_phy_init(struct xgbe_prv_data *pdata)
{
	pdata->phy_link = -1;
	pdata->phy_speed = SPEED_UNKNOWN;

	return pdata->phy_if.phy_reset(pdata);
}

static int xgbe_start(struct xgbe_prv_data *pdata)
{
	struct xgbe_hw_if *hw_if = &pdata->hw_if;
	struct xgbe_phy_if *phy_if = &pdata->phy_if;
	int ret;

	DBGPR("-->xgbe_start\n");

	hw_if->init(pdata);

	ret = phy_if->phy_start(pdata);
	if (ret)
		goto err_phy;

	ret = xgbe_request_irqs(pdata);
	if (ret)
		goto err_napi;

	hw_if->enable_tx(pdata);
	hw_if->enable_rx(pdata);

	xgbe_enable_rx_tx_ints(pdata);

	xgbe_start_timers(pdata);
	taskqueue_enqueue(pdata->dev_workqueue, &pdata->service_work);

	DBGPR("<--xgbe_start\n");

	return 0;

err_napi:
	phy_if->phy_stop(pdata);

err_phy:
	hw_if->exit(pdata);

	return ret;
}

static void xgbe_stop(struct xgbe_prv_data *pdata)
{
	struct xgbe_hw_if *hw_if = &pdata->hw_if;
	struct xgbe_phy_if *phy_if = &pdata->phy_if;

	DBGPR("-->xgbe_stop\n");

	xgbe_stop_timers(pdata);
	taskqueue_drain_all(pdata->dev_workqueue);

	hw_if->disable_tx(pdata);
	hw_if->disable_rx(pdata);

	xgbe_free_irqs(pdata);

	phy_if->phy_stop(pdata);

	hw_if->exit(pdata);

	DBGPR("<--xgbe_stop\n");
}

static void xgbe_restart_dev(struct xgbe_prv_data *pdata)
{
	DBGPR("-->xgbe_restart_dev\n");

	/* If not running, "restart" will happen on open */
	if ((pdata->netdev->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return;

	xgbe_stop(pdata);

	xgbe_free_tx_data(pdata);
	xgbe_free_rx_data(pdata);

	xgbe_start(pdata);

	DBGPR("<--xgbe_restart_dev\n");
}

static void xgbe_restart(void *ctx, int pending)
{
	struct xgbe_prv_data *pdata = ctx;

	xgbe_restart_dev(pdata);
}

static void xgbe_packet_info(struct xgbe_prv_data *pdata,
			     struct xgbe_ring *ring, struct mbuf *m0,
			     struct xgbe_packet_data *packet)
{
	struct mbuf *m;
	unsigned int len;

	packet->m = m0;

	packet->rdesc_count = 0;

	packet->tx_packets = 1;
	packet->tx_bytes = m_length(m0, NULL);

	for (m = m0; m != NULL; m = m->m_next) {
		for (len = m->m_len; len != 0;) {
			packet->rdesc_count++;
			len -= MIN(len, XGBE_TX_MAX_BUF_SIZE);
		}
	}
}

int xgbe_open(struct ifnet *netdev)
{
	struct xgbe_prv_data *pdata = netdev->if_softc;
	struct xgbe_desc_if *desc_if = &pdata->desc_if;
	int ret;

	DBGPR("-->xgbe_open\n");

	/* Initialize the phy */
	ret = xgbe_phy_init(pdata);
	if (ret)
		return ret;

	/* Calculate the Rx buffer size before allocating rings */
	ret = xgbe_calc_rx_buf_size(netdev, if_getmtu(netdev));
	if (ret < 0) {
		goto err_ptpclk;
	}
	pdata->rx_buf_size = ret;

	/* Allocate the channel and ring structures */
	ret = xgbe_alloc_channels(pdata);
	if (ret) {
		printf("xgbe_alloc_channels failed\n");
		goto err_ptpclk;
	}

	/* Allocate the ring descriptors and buffers */
	ret = desc_if->alloc_ring_resources(pdata);
	if (ret) {
		printf("desc_if->alloc_ring_resources failed\n");
		goto err_channels;
	}

	TASK_INIT(&pdata->service_work, 0, xgbe_service, pdata);
	TASK_INIT(&pdata->restart_work, 0, xgbe_restart, pdata);
	xgbe_init_timers(pdata);

	ret = xgbe_start(pdata);
	if (ret)
		goto err_rings;

	clear_bit(XGBE_DOWN, &pdata->dev_state);

	DBGPR("<--xgbe_open\n");

	return 0;

err_rings:
	desc_if->free_ring_resources(pdata);

err_channels:
	xgbe_free_channels(pdata);

err_ptpclk:

	return ret;
}

int xgbe_close(struct ifnet *netdev)
{
	struct xgbe_prv_data *pdata = netdev->if_softc;
	struct xgbe_desc_if *desc_if = &pdata->desc_if;

	DBGPR("-->xgbe_close\n");

	/* Stop the device */
	xgbe_stop(pdata);

	/* Free the ring descriptors and buffers */
	desc_if->free_ring_resources(pdata);

	/* Free the channel and ring structures */
	xgbe_free_channels(pdata);

	set_bit(XGBE_DOWN, &pdata->dev_state);

	DBGPR("<--xgbe_close\n");

	return 0;
}

int xgbe_xmit(struct ifnet *ifp, struct mbuf *m)
{
	struct xgbe_prv_data *pdata = ifp->if_softc;
	struct xgbe_hw_if *hw_if = &pdata->hw_if;
	struct xgbe_desc_if *desc_if = &pdata->desc_if;
	struct xgbe_channel *channel;
	struct xgbe_ring *ring;
	struct xgbe_packet_data *packet;
	int ret;

	M_ASSERTPKTHDR(m);
	MPASS(m->m_nextpkt == NULL);

	if (__predict_false(test_bit(XGBE_DOWN, &pdata->dev_state) ||
	    !pdata->phy.link)) {
		m_freem(m);
		return (ENETDOWN);
	}

	channel = pdata->channel;
	ring = channel->tx_ring;
	packet = &ring->packet_data;

	/* Calculate preliminary packet info */
	memset(packet, 0, sizeof(*packet));
	xgbe_packet_info(pdata, ring, m, packet);

	/* Check that there are enough descriptors available */
	ret = xgbe_maybe_stop_tx_queue(channel, ring, packet->rdesc_count);
	if (ret)
		goto tx_netdev_return;

	if (!desc_if->map_tx_skb(channel, m)) {
		goto tx_netdev_return;
	}

	/* Configure required descriptor fields for transmission */
	hw_if->dev_xmit(channel);

	return 0;

tx_netdev_return:
	m_free(m);

	return 0;
}

int xgbe_change_mtu(struct ifnet *netdev, int mtu)
{
	struct xgbe_prv_data *pdata = netdev->if_softc;
	int ret;

	DBGPR("-->xgbe_change_mtu\n");

	ret = xgbe_calc_rx_buf_size(netdev, mtu);
	if (ret < 0)
		return -ret;

	pdata->rx_buf_size = ret;
	netdev->if_mtu = mtu;

	xgbe_restart_dev(pdata);

	DBGPR("<--xgbe_change_mtu\n");

	return 0;
}

static void xgbe_rx_refresh(struct xgbe_channel *channel)
{
	struct xgbe_prv_data *pdata = channel->pdata;
	struct xgbe_hw_if *hw_if = &pdata->hw_if;
	struct xgbe_desc_if *desc_if = &pdata->desc_if;
	struct xgbe_ring *ring = channel->rx_ring;
	struct xgbe_ring_data *rdata;

	while (ring->dirty != ring->cur) {
		rdata = XGBE_GET_DESC_DATA(ring, ring->dirty);

		/* Reset rdata values */
		desc_if->unmap_rdata(pdata, rdata);

		if (desc_if->map_rx_buffer(pdata, ring, rdata))
			break;

		hw_if->rx_desc_reset(pdata, rdata, ring->dirty);

		ring->dirty++;
	}

	/* Make sure everything is written before the register write */
	dsb(sy);

	/* Update the Rx Tail Pointer Register with address of
	 * the last cleaned entry */
	rdata = XGBE_GET_DESC_DATA(ring, ring->dirty - 1);
	XGMAC_DMA_IOWRITE(channel, DMA_CH_RDTR_LO,
			  lower_32_bits(rdata->rdata_paddr));
}

static int xgbe_tx_poll(struct xgbe_channel *channel)
{
	struct xgbe_prv_data *pdata = channel->pdata;
	struct xgbe_hw_if *hw_if = &pdata->hw_if;
	struct xgbe_desc_if *desc_if = &pdata->desc_if;
	struct xgbe_ring *ring = channel->tx_ring;
	struct xgbe_ring_data *rdata;
	struct xgbe_ring_desc *rdesc;
	int processed = 0;
	unsigned int cur;

	DBGPR("-->xgbe_tx_poll\n");

	/* Nothing to do if there isn't a Tx ring for this channel */
	if (!ring)
		return 0;

	cur = ring->cur;

	/* Be sure we get ring->cur before accessing descriptor data */
	dsb(sy);

	while ((processed < XGBE_TX_DESC_MAX_PROC) &&
	       (ring->dirty != cur)) {
		rdata = XGBE_GET_DESC_DATA(ring, ring->dirty);
		rdesc = rdata->rdesc;

		if (!hw_if->tx_complete(rdesc))
			break;

		/* Make sure descriptor fields are read after reading the OWN
		 * bit */
		dsb(sy);

		/* Free the SKB and reset the descriptor for re-use */
		desc_if->unmap_rdata(pdata, rdata);
		hw_if->tx_desc_reset(rdata);

		processed++;
		ring->dirty++;
	}

	if (!processed)
		return 0;

	DBGPR("<--xgbe_tx_poll: processed=%d\n", processed);

	return processed;
}

static int xgbe_rx_poll(struct xgbe_channel *channel, int budget)
{
	struct xgbe_prv_data *pdata = channel->pdata;
	struct xgbe_hw_if *hw_if = &pdata->hw_if;
	struct xgbe_ring *ring = channel->rx_ring;
	struct xgbe_ring_data *rdata;
	struct xgbe_packet_data *packet;
	struct ifnet *ifp = pdata->netdev;
	struct mbuf *m;
	unsigned int incomplete, context_next;
	unsigned int received = 0;
	int packet_count = 0;

	DBGPR("-->xgbe_rx_poll: budget=%d\n", budget);

	/* Nothing to do if there isn't a Rx ring for this channel */
	if (!ring)
		return 0;

	incomplete = 0;
	context_next = 0;

	rdata = XGBE_GET_DESC_DATA(ring, ring->cur);
	packet = &ring->packet_data;
	while (packet_count < budget) {
		DBGPR("  cur = %d\n", ring->cur);

read_again:
		rdata = XGBE_GET_DESC_DATA(ring, ring->cur);

		if (xgbe_rx_dirty_desc(ring) > (XGBE_RX_DESC_CNT >> 3))
			xgbe_rx_refresh(channel);

		if (hw_if->dev_read(channel))
			break;

		m = rdata->mb;

		received++;
		ring->cur++;

		incomplete = XGMAC_GET_BITS(packet->attributes,
					    RX_PACKET_ATTRIBUTES,
					    INCOMPLETE);
		context_next = XGMAC_GET_BITS(packet->attributes,
					      RX_PACKET_ATTRIBUTES,
					      CONTEXT_NEXT);

		/* Earlier error, just drain the remaining data */
		if (incomplete || context_next) {
			goto read_again;
		}

		if (packet->errors) {
			rdata->mbuf_free = 1;
			goto next_packet;
		}
		rdata->mb = NULL;

		m->m_pkthdr.len = rdata->rx.hdr_len + rdata->rx.len;
		if (rdata->rx.hdr_len != 0) {
			m->m_len = rdata->rx.hdr_len;
			m->m_next->m_len = rdata->rx.len;
		} else {
			m->m_len = rdata->rx.len;
			m_freem(m->m_next);
			m->m_next = NULL;
		}
		if_setrcvif(m, ifp);
		if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);

		ifp->if_input(ifp, m);

next_packet:
		packet_count++;
	}

	DBGPR("<--xgbe_rx_poll: packet_count = %d\n", packet_count);

	return packet_count;
}

static int xgbe_one_poll(struct xgbe_channel *channel, int budget)
{
	int processed = 0;

	DBGPR("-->xgbe_one_poll: budget=%d\n", budget);

	/* Cleanup Tx ring first */
	xgbe_tx_poll(channel);

	/* Process Rx ring next */
	processed = xgbe_rx_poll(channel, budget);

	DBGPR("<--xgbe_one_poll: received = %d\n", processed);

	return processed;
}

static int xgbe_all_poll(struct xgbe_prv_data *pdata, int budget)
{
	struct xgbe_channel *channel;
	int ring_budget;
	int processed, last_processed;
	unsigned int i;

	DBGPR("-->xgbe_all_poll: budget=%d\n", budget);

	processed = 0;
	ring_budget = budget / pdata->rx_ring_count;
	do {
		last_processed = processed;

		channel = pdata->channel;
		for (i = 0; i < pdata->channel_count; i++, channel++) {
			/* Cleanup Tx ring first */
			xgbe_tx_poll(channel);

			/* Process Rx ring next */
			if (ring_budget > (budget - processed))
				ring_budget = budget - processed;
			processed += xgbe_rx_poll(channel, ring_budget);
		}
	} while ((processed < budget) && (processed != last_processed));

	DBGPR("<--xgbe_all_poll: received = %d\n", processed);

	return processed;
}
