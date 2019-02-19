/*
 *  Portions Copyright 2019 Alan Johnston
 *
 *  Portions Copyright (C) 2018 Jonathan Brandenburg
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Warning: This file is automatically generated by AX-RadioLAB.
   Manual changes are overwritten! */

#include "config.h"

#include "../axradio/axradioinit.h"
#include "../spi/ax5043spi.h"
#include "../crc/crc.h"

// TX: fcarrier=441.000MHz dev=  0.500kHz br=  1.200kBit/s pwr= 15.0dBm
// RX: fcarrier=441.000MHz bw=  1.800kHz br=  1.200kBit/s

 void ax5043_set_registers(void)
{
	ax5043WriteReg(AX5043_MODULATION, 0x04); // PSK was 0x08);
	ax5043WriteReg(AX5043_ENCODING, 0x02); // Differential encoding was 0);
	ax5043WriteReg(AX5043_FRAMING, 0x66); // Framing from gen?  0x24);
	ax5043WriteReg(AX5043_FEC, 0x13);
	ax5043WriteReg(AX5043_PINFUNCSYSCLK, 0x01);
	ax5043WriteReg(AX5043_PINFUNCDCLK, 0x01);
	ax5043WriteReg(AX5043_PINFUNCDATA, 0x01);
	ax5043WriteReg(AX5043_PINFUNCANTSEL, 0x01);
	ax5043WriteReg(AX5043_PINFUNCPWRAMP, 0x07);
	ax5043WriteReg(AX5043_WAKEUPXOEARLY, 0x01);
	ax5043WriteReg(AX5043_IFFREQ1, 0x02);
	ax5043WriteReg(AX5043_IFFREQ0, 0x04);
	ax5043WriteReg(AX5043_DECIMATION, 0x6F);
	ax5043WriteReg(AX5043_RXDATARATE2, 0x00);
	ax5043WriteReg(AX5043_RXDATARATE1, 0x3C);
	ax5043WriteReg(AX5043_RXDATARATE0, 0x0F);
	ax5043WriteReg(AX5043_MAXDROFFSET2, 0x00);
	ax5043WriteReg(AX5043_MAXDROFFSET1, 0x00);
	ax5043WriteReg(AX5043_MAXDROFFSET0, 0x00);
	ax5043WriteReg(AX5043_MAXRFOFFSET2, 0x80);
	ax5043WriteReg(AX5043_MAXRFOFFSET1, 0x03);
	ax5043WriteReg(AX5043_MAXRFOFFSET0, 0x9B);
	ax5043WriteReg(AX5043_FSKDMAX1, 0x00);
	ax5043WriteReg(AX5043_FSKDMAX0, 0xA6);
	ax5043WriteReg(AX5043_FSKDMIN1, 0xFF);
	ax5043WriteReg(AX5043_FSKDMIN0, 0x5A);
	ax5043WriteReg(AX5043_AMPLFILTER, 0x00);
	ax5043WriteReg(AX5043_RXPARAMSETS, 0xF4);
	ax5043WriteReg(AX5043_AGCGAIN0, 0xE8);
	ax5043WriteReg(AX5043_AGCTARGET0, 0x84);
	ax5043WriteReg(AX5043_TIMEGAIN0, 0xF8);
	ax5043WriteReg(AX5043_DRGAIN0, 0xF2);
	ax5043WriteReg(AX5043_PHASEGAIN0, 0xC3);
	ax5043WriteReg(AX5043_FREQUENCYGAINA0, 0x0F);
	ax5043WriteReg(AX5043_FREQUENCYGAINB0, 0x1F);
	ax5043WriteReg(AX5043_FREQUENCYGAINC0, 0x0A);
	ax5043WriteReg(AX5043_FREQUENCYGAIND0, 0x0A);
	ax5043WriteReg(AX5043_AMPLITUDEGAIN0, 0x06);
	ax5043WriteReg(AX5043_FREQDEV10, 0x00);
	ax5043WriteReg(AX5043_FREQDEV00, 0x00);
	ax5043WriteReg(AX5043_BBOFFSRES0, 0x00);
	ax5043WriteReg(AX5043_AGCGAIN1, 0xE8);
	ax5043WriteReg(AX5043_AGCTARGET1, 0x84);
	ax5043WriteReg(AX5043_AGCAHYST1, 0x00);
	ax5043WriteReg(AX5043_AGCMINMAX1, 0x00);
	ax5043WriteReg(AX5043_TIMEGAIN1, 0xF6);
	ax5043WriteReg(AX5043_DRGAIN1, 0xF1);
	ax5043WriteReg(AX5043_PHASEGAIN1, 0xC3);
	ax5043WriteReg(AX5043_FREQUENCYGAINA1, 0x0F);
	ax5043WriteReg(AX5043_FREQUENCYGAINB1, 0x1F);
	ax5043WriteReg(AX5043_FREQUENCYGAINC1, 0x0A);
	ax5043WriteReg(AX5043_FREQUENCYGAIND1, 0x0A);
	ax5043WriteReg(AX5043_AMPLITUDEGAIN1, 0x06);
	ax5043WriteReg(AX5043_FREQDEV11, 0x00);
	ax5043WriteReg(AX5043_FREQDEV01, 0x4B);
	ax5043WriteReg(AX5043_FOURFSK1, 0x16);
	ax5043WriteReg(AX5043_BBOFFSRES1, 0x00);
	ax5043WriteReg(AX5043_AGCGAIN3, 0xFF);
	ax5043WriteReg(AX5043_AGCTARGET3, 0x84);
	ax5043WriteReg(AX5043_AGCAHYST3, 0x00);
	ax5043WriteReg(AX5043_AGCMINMAX3, 0x00);
	ax5043WriteReg(AX5043_TIMEGAIN3, 0xF5);
	ax5043WriteReg(AX5043_DRGAIN3, 0xF0);
	ax5043WriteReg(AX5043_PHASEGAIN3, 0xC3);
	ax5043WriteReg(AX5043_FREQUENCYGAINA3, 0x0F);
	ax5043WriteReg(AX5043_FREQUENCYGAINB3, 0x1F);
	ax5043WriteReg(AX5043_FREQUENCYGAINC3, 0x0D);
	ax5043WriteReg(AX5043_FREQUENCYGAIND3, 0x0D);
	ax5043WriteReg(AX5043_AMPLITUDEGAIN3, 0x06);
	ax5043WriteReg(AX5043_FREQDEV13, 0x00);
	ax5043WriteReg(AX5043_FREQDEV03, 0x4B);
	ax5043WriteReg(AX5043_FOURFSK3, 0x16);
	ax5043WriteReg(AX5043_BBOFFSRES3, 0x00);
	ax5043WriteReg(AX5043_MODCFGF, 0x00);
	ax5043WriteReg(AX5043_FSKDEV2, 0x00);
	ax5043WriteReg(AX5043_FSKDEV1, 0x02);
	ax5043WriteReg(AX5043_FSKDEV0, 0x0C);
	ax5043WriteReg(AX5043_MODCFGA, 0x05);
	ax5043WriteReg(AX5043_TXRATE2, 0x00);
	ax5043WriteReg(AX5043_TXRATE1, 0x04); // 1200bps 0x13);
	ax5043WriteReg(AX5043_TXRATE0, 0xE4); //         0xA9);
	ax5043WriteReg(AX5043_TXPWRCOEFFB1, 0x0F);
	ax5043WriteReg(AX5043_TXPWRCOEFFB0, 0xFF);
	ax5043WriteReg(AX5043_PLLVCOI, 0x99);
	ax5043WriteReg(AX5043_PLLRNGCLK, 0x03);
	ax5043WriteReg(AX5043_BBTUNE, 0x0F);
	ax5043WriteReg(AX5043_BBOFFSCAP, 0x77);
	ax5043WriteReg(AX5043_PKTADDRCFG, 0x00);
	ax5043WriteReg(AX5043_PKTLENCFG, 0x00);
	ax5043WriteReg(AX5043_PKTLENOFFSET, 0x08);
	ax5043WriteReg(AX5043_PKTMAXLEN, 0xC8);
	ax5043WriteReg(AX5043_MATCH0PAT3, 0xA3);
	ax5043WriteReg(AX5043_MATCH0PAT2, 0xA3);
	ax5043WriteReg(AX5043_MATCH0PAT1, 0x00);
	ax5043WriteReg(AX5043_MATCH0PAT0, 0x00);
	ax5043WriteReg(AX5043_MATCH1PAT1, 0x55);
	ax5043WriteReg(AX5043_MATCH1PAT0, 0x55);
	ax5043WriteReg(AX5043_MATCH1LEN, 0x8A);
	ax5043WriteReg(AX5043_MATCH1MAX, 0x0A);
	ax5043WriteReg(AX5043_TMGTXBOOST, 0x32);
	ax5043WriteReg(AX5043_TMGTXSETTLE, 0x14);
	ax5043WriteReg(AX5043_TMGRXBOOST, 0x32);
	ax5043WriteReg(AX5043_TMGRXSETTLE, 0x14);
	ax5043WriteReg(AX5043_TMGRXOFFSACQ, 0x00);
	ax5043WriteReg(AX5043_TMGRXCOARSEAGC, 0x73);
	ax5043WriteReg(AX5043_TMGRXRSSI, 0x03);
	ax5043WriteReg(AX5043_TMGRXPREAMBLE2, 0x1A);
	ax5043WriteReg(AX5043_RSSIABSTHR, 0xDC);
	ax5043WriteReg(AX5043_BGNDRSSITHR, 0x00);
	ax5043WriteReg(AX5043_PKTCHUNKSIZE, 0x0D);
	ax5043WriteReg(AX5043_PKTACCEPTFLAGS, 0x20);
	ax5043WriteReg(AX5043_DACVALUE1, 0x00);
	ax5043WriteReg(AX5043_DACVALUE0, 0x00);
	ax5043WriteReg(AX5043_DACCONFIG, 0x00);
	ax5043WriteReg(AX5043_REF, 0x03);
	ax5043WriteReg(AX5043_XTALOSC, 0x04);
	ax5043WriteReg(AX5043_XTALAMPL, 0x00);
	ax5043WriteReg(AX5043_0xF1C, 0x07);
	ax5043WriteReg(AX5043_0xF21, 0x68);
	ax5043WriteReg(AX5043_0xF22, 0xFF);
	ax5043WriteReg(AX5043_0xF23, 0x84);
	ax5043WriteReg(AX5043_0xF26, 0x98);
	ax5043WriteReg(AX5043_0xF34, 0x28);
	ax5043WriteReg(AX5043_0xF35, 0x10);
	ax5043WriteReg(AX5043_0xF44, 0x25);
}


 void ax5043_set_registers_tx(void)
{
	ax5043WriteReg(AX5043_PLLLOOP, 0x0B);
	ax5043WriteReg(AX5043_PLLCPI, 0x10);
	ax5043WriteReg(AX5043_PLLVCODIV, 0x24);
	ax5043WriteReg(AX5043_XTALCAP, 0x00);
	ax5043WriteReg(AX5043_0xF00, 0x0F);
	ax5043WriteReg(AX5043_0xF18, 0x06);
}


 void ax5043_set_registers_rx(void)
{
	ax5043WriteReg(AX5043_PLLLOOP, 0x0B);
	ax5043WriteReg(AX5043_PLLCPI, 0x10);
	ax5043WriteReg(AX5043_PLLVCODIV, 0x24);
	ax5043WriteReg(AX5043_XTALCAP, 0x00);
	ax5043WriteReg(AX5043_0xF00, 0x0F);
	ax5043WriteReg(AX5043_0xF18, 0x02);
}


 void ax5043_set_registers_rxwor(void)
{
	ax5043WriteReg(AX5043_TMGRXAGC, 0x15);
	ax5043WriteReg(AX5043_TMGRXPREAMBLE1, 0x19);
	ax5043WriteReg(AX5043_PKTMISCFLAGS, 0x03);
}


 void ax5043_set_registers_rxcont(void)
{
	ax5043WriteReg(AX5043_TMGRXAGC, 0x00);
	ax5043WriteReg(AX5043_TMGRXPREAMBLE1, 0x00);
	ax5043WriteReg(AX5043_PKTMISCFLAGS, 0x00);
}


 void ax5043_set_registers_rxcont_singleparamset(void)
{
	ax5043WriteReg(AX5043_RXPARAMSETS, 0xFF);
	ax5043WriteReg(AX5043_FREQDEV13, 0x00);
	ax5043WriteReg(AX5043_FREQDEV03, 0x00);
	ax5043WriteReg(AX5043_AGCGAIN3, 0xE8);
}


#define MUL8_16(x,y) ((uint8_t)((x)&0xff)*(uint16_t)(uint8_t)((y)&0xff))

#define CONSTMULFIX24(x)					\
	if (f >= 0) {						\
		uint32_t r = MUL8_16((x)>>16,f);		\
		r += MUL8_16((x)>>8,f>>8);			\
		r += MUL8_16((x),f>>16);			\
		r >>= 8;					\
		r += MUL8_16((x)>>24,f);			\
		r += MUL8_16((x)>>16,f>>8);			\
		r += MUL8_16((x)>>8,f>>16);			\
		r += MUL8_16((x),f>>24);			\
		r += ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>16,f>>16))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>8,f>>24))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>24,f>>16))<<16;	\
		r += ((uint32_t)MUL8_16((x)>>16,f>>24))<<16;	\
		r += ((uint32_t)MUL8_16((x)>>24,f>>24))<<24;	\
		return r;					\
	}							\
	{							\
		int32_t r;					\
		f = -f;						\
		r = -(uint32_t)MUL8_16((x)>>16,f);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>8);		\
		r -= (uint32_t)MUL8_16((x),f>>16);		\
		r >>= 8;					\
		r -= (uint32_t)MUL8_16((x)>>24,f);		\
		r -= (uint32_t)MUL8_16((x)>>16,f>>8);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>16);		\
		r -= (uint32_t)MUL8_16((x),f>>24);		\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>16,f>>16))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>8,f>>24))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>16))<<16;	\
		r -= ((uint32_t)MUL8_16((x)>>16,f>>24))<<16;	\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>24))<<24;	\
		return r;					\
	}

#define CONSTMULFIX16(x)					\
	if (f >= 0) {						\
		uint32_t r = MUL8_16((x)>>16,f);		\
		r += MUL8_16((x)>>8,f>>8);			\
		r >>= 8;					\
		r += MUL8_16((x)>>24,f);			\
		r += MUL8_16((x)>>16,f>>8);			\
		r += ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		return r;					\
	}							\
	{							\
		int32_t r;					\
		f = -f;						\
		r = -(uint32_t)MUL8_16((x)>>16,f);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>8);		\
		r >>= 8;					\
		r -= (uint32_t)MUL8_16((x)>>24,f);		\
		r -= (uint32_t)MUL8_16((x)>>16,f>>8);		\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		return r;					\
	}

 int32_t axradio_conv_freq_fromhz(int32_t f)
{
	/* scale by 1.048576 (true 1.048576) */
	CONSTMULFIX24(0x10c6f7a);
	/* scale by 0.349525 (true 0.349525) */
//	CONSTMULFIX24(0x597a7e);
}

 int32_t axradio_conv_freq_tohz(int32_t f)
{
	/* scale by 0.953674 (true 0.953674) */
	CONSTMULFIX24(0xf42400);
	/* scale by 2.861023 (true 2.861023) */
//	CONSTMULFIX24(0x2dc6c00);
}

const uint8_t axradio_phy_innerfreqloop = 0;

 int32_t axradio_conv_freq_fromreg(int32_t f)
{
	/* scale by 1.000000 (true 1.000000) */
	CONSTMULFIX16(0x1000000);
}

 int32_t axradio_conv_timeinterval_totimer0(int32_t dt)
{
	/* scale by 0.032776 (true 0.032768) */
	int32_t r;
	dt >>= 5;
	r = dt;
	dt >>= 4;
	r += dt;
	dt >>= 2;
	r -= dt;
	dt >>= 3;
	r += dt;
	return r;
}

 uint8_t axradio_byteconv(uint8_t b)
{
	return b;
}


 void axradio_byteconv_buffer(uint8_t *buf __attribute__((unused)), uint16_t buflen __attribute__((unused)))
{
}

 uint16_t axradio_framing_check_crc(uint8_t *pkt, uint16_t cnt)
{
	if (crc_crc16(pkt, cnt, 0xFFFF) != 0xB001)
		return 0;
	return cnt;
}

 uint16_t axradio_framing_append_crc(uint8_t *pkt, uint16_t cnt)
{
	uint16_t s = 0xFFFF;
	s = crc_crc16(pkt, cnt, s);
	pkt += cnt;
	*pkt++ = ~(uint8_t)(s);
	*pkt++ = ~(uint8_t)(s >> 8);
	return cnt + 2;
}





// physical layer
const uint8_t axradio_phy_pn9 = 0;
const uint8_t axradio_phy_nrchannels = 1;
const uint32_t axradio_phy_chanfreq[1] = { 0x1b800001 };
const uint8_t axradio_phy_chanpllrnginit[1] = { 0x09 };
const uint8_t axradio_phy_chanvcoiinit[1] = { 0x99 };
uint8_t axradio_phy_chanpllrng[1];
uint8_t axradio_phy_chanvcoi[1];
const uint8_t axradio_phy_vcocalib = 0;
const int32_t axradio_phy_maxfreqoffset = 2768;
const int8_t axradio_phy_rssioffset = 64;
// axradio_phy_rssioffset is added to AX5043_RSSIREFERENCE and subtracted from chip RSSI value to prevent overflows (8bit RSSI only goes down to -128)
// axradio_phy_rssioffset is also added to AX5043_RSSIABSTHR
const int8_t axradio_phy_rssireference = (int8_t)(0xF8 + 64);
const int8_t axradio_phy_channelbusy = -100 + 64;
const uint16_t axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t axradio_phy_cs_enabled = 0;
const uint8_t axradio_phy_lbt_retries = 0;
const uint8_t axradio_phy_lbt_forcetx = 0;
const uint16_t axradio_phy_preamble_wor_longlen = 1; // 2; // wor_longlen + wor_len totals to 240.0ms plus 112bits
const uint16_t axradio_phy_preamble_wor_len = 64; // 176;
const uint16_t axradio_phy_preamble_longlen = 0;
const uint16_t axradio_phy_preamble_len = 32; // 112;
const uint8_t axradio_phy_preamble_byte = 0x05;
const uint8_t axradio_phy_preamble_flags = 0x38;
const uint8_t axradio_phy_preamble_appendbits = 0;
const uint8_t axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t axradio_framing_maclen = 2; // 1;
const uint8_t axradio_framing_addrlen = 2;
const uint8_t axradio_framing_destaddrpos = 0;
const uint8_t axradio_framing_sourceaddrpos = 0xff;
const uint8_t axradio_framing_lenpos = 0;
const uint8_t axradio_framing_lenoffs = 8;
const uint8_t axradio_framing_lenmask = 0x00;
const uint8_t axradio_framing_swcrclen = 0;

const uint8_t axradio_framing_synclen = 16; // 32;
const uint8_t axradio_framing_syncword[] = { 0xa3, 0xa3, 0xcc, 0xaa }; // , 0xaa, 0xcc, 0xaa};
const uint8_t axradio_framing_syncflags = 0x18;
const uint8_t axradio_framing_enable_sfdcallback = 0;

const uint32_t axradio_framing_ack_timeout = 87; // 98.9ms in wtimer0 units (640Hz)
const uint32_t axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t axradio_framing_ack_retransmissions = 0;
const uint8_t axradio_framing_ack_seqnrpos = 0xff;

const uint8_t axradio_framing_minpayloadlen = 1; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
//WOR
const uint16_t axradio_wor_period = 128;

// synchronous mode
const uint32_t axradio_sync_period = 32768; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t axradio_sync_xoscstartup = 49;
const uint32_t axradio_sync_slave_syncwindow = 98304; // 3.000s
const uint32_t axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t axradio_sync_slave_maxperiod = 2020; // in (2^SYNC_K1) * wtimer0 units
const uint8_t axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t axradio_sync_slave_nrrx = 3;
const uint32_t axradio_sync_slave_rxadvance[] = { 1988, 1973, 2234 };// 55.918ms, 54.788ms, 57.167ms
const uint32_t axradio_sync_slave_rxwindow[] = { 2041, 2011, 2533 }; // 56.346ms, 54.086ms, 58.844ms
const uint32_t axradio_sync_slave_rxtimeout = 3987; // 92.5ms, maximum duration of a packet
