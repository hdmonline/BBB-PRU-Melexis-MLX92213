/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SYS_MCSPI_H_
#define _SYS_MCSPI_H_

/* SYS MCSPI register set */
typedef struct {

	/* SYS_MCSPI_HL_REV register bit field */
	union {
		volatile uint32_t HL_REV;

		volatile struct {
			uint32_t REVISION : 32; // 31:0
		} HL_REV_bit;
	}; // 0x0

	/* SYS_MCSPI_HL_HWINFO register bit field */
	union {
		volatile uint32_t HL_HWINFO;

		volatile struct {
			uint32_t USEFIFO : 1; // 0
			uint32_t FFNBYTE : 5; // 5:1
			uint32_t RETMODE : 1; // 6
			uint32_t rsvd7 : 25; // 31:7
		} HL_HWINFO_bit;
	}; // 0x4

	uint8_t rsvd8[8]; // 0x8 - 0xf

	/* SYS_MCSPI_HL_SYSCONFIG register bit field */
	union {
		volatile uint32_t HL_SYSCONFIG;

		volatile struct {
			uint32_t SOFTRESET : 1; // 0
			uint32_t FREEEMU : 1; // 1
			uint32_t IDLEMODE : 2; // 3:2
			uint32_t rsvd4 : 28; // 31:4
		} HL_SYSCONFIG_bit;
	}; // 0x10

	uint8_t rsvd14[236]; // 0x14 - 0xff

	/* SYS_MCSPI_REVISION register bit field */
	union {
		volatile uint32_t REVISION;

		volatile struct {
			uint32_t REVISION : 32; // 31:0
		} REVISION_bit;
	}; // 0x100

	uint8_t rsvd104[12]; // 0x104 - 0x10f

	/* SYS_MCSPI_SYSCONFIG register bit field */
	union {
		volatile uint32_t SYSCONFIG;

		volatile struct {
			uint32_t AUTOIDLE : 1; // 0
			uint32_t SOFTRESET : 1; // 1
			uint32_t ENAWAKEUP : 1; // 2
			uint32_t SIDLEMODE : 2; // 4:3
			uint32_t rsvd5 : 3; // 7:5
			uint32_t CLOCKACTIVITY : 2; // 9:8
			uint32_t rsvd10 : 22; // 31:10
		} SYSCONFIG_bit;
	}; // 0x110

	/* SYS_MCSPI_SYSSTATUS register bit field */
	union {
		volatile uint32_t SYSSTATUS;

		volatile struct {
			uint32_t RESETDONE : 1; // 0
			uint32_t rsvd1 : 31; // 31:1
		} SYSSTATUS_bit;
	}; // 0x114

	/* SYS_MCSPI_IRQSTATUS register bit field */
	union {
		volatile uint32_t IRQSTATUS;

		volatile struct {
			uint32_t TX0_EMPTY : 1; // 0
			uint32_t TX0_UNDERFLOW : 1; // 1
			uint32_t RX0_FULL : 1; // 2
			uint32_t RX0_OVERFLOW : 1; // 3
			uint32_t TX1_EMPTY : 1; // 4
			uint32_t TX1_UNDERFLOW : 1; // 5
			uint32_t RX1_FULL : 1; // 6
			uint32_t rsvd7 : 1; // 7
			uint32_t TX2_EMPTY : 1; // 8
			uint32_t TX2_UNDERFLOW : 1; // 9
			uint32_t RX2_FULL : 1; // 10
			uint32_t rsvd11 : 1; // 11
			uint32_t TX3_EMPTY : 1; // 12
			uint32_t TX3_UNDERFLOW : 1; // 13
			uint32_t RX3_FULL : 1; // 14
			uint32_t rsvd15 : 1; // 15
			uint32_t WKS : 1; // 16
			uint32_t EOW : 1; // 17
			uint32_t rsvd18 : 14; // 31:18
		} IRQSTATUS_bit;
	}; // 0x118

	/* SYS_MCSPI_IRQENABLE register bit field */
	union {
		volatile uint32_t IRQENABLE;

		volatile struct {
			uint32_t TX0_EMPTY_ENABLE : 1; // 0
			uint32_t TX0_UNDERFLOW_ENABLE : 1; // 1
			uint32_t RX0_FULL_ENABLE : 1; // 2
			uint32_t RX0_OVERFLOW_ENABLE : 1; // 3
			uint32_t TX1_EMPTY_ENABLE : 1; // 4
			uint32_t TX1_UNDERFLOW_ENABLE : 1; // 5
			uint32_t RX1_FULL_ENABLE : 1; // 6
			uint32_t rsvd7 : 1; // 7
			uint32_t TX2_EMPTY_ENABLE : 1; // 8
			uint32_t TX2_UNDERFLOW_ENABLE : 1; // 9
			uint32_t RX2_FULL_ENABLE : 1; // 10
			uint32_t rsvd11 : 1; // 11
			uint32_t TX3_EMPTY_ENABLE : 1; // 12
			uint32_t TX3_UNDERFLOW_ENABLE : 1; // 13
			uint32_t RX3_FULL_ENABLE : 1; // 14
			uint32_t rsvd15 : 1; // 15
			uint32_t WKE : 1; // 16
			uint32_t EOW_ENABLE : 1; // 17
			uint32_t rsvd18 : 14; // 31:18
		} IRQENABLE_bit;
	}; // 0x11c

	/* SYS_MCSPI_WAKEUPENABLE register bit field */
	union {
		volatile uint32_t WAKEUPENABLE;

		volatile struct {
			uint32_t WKEN : 1; // 0
			uint32_t rsvd1 : 31; // 31:1
		} WAKEUPENABLE_bit;
	}; // 0x120

	/* SYS_MCSPI_SYST register bit field */
	union {
		volatile uint32_t SYST;

		volatile struct {
			uint32_t SPIEN_0 : 1; // 0
			uint32_t SPIEN_1 : 1; // 1
			uint32_t SPIEN_2 : 1; // 2
			uint32_t SPIEN_3 : 1; // 3
			uint32_t SPIDAT_0 : 1; // 4
			uint32_t SPIDAT_1 : 1; // 5
			uint32_t SPICLK : 1; // 6
			uint32_t WAKD : 1; // 7
			uint32_t SPIDATDIR0 : 1; // 8
			uint32_t SPIDATDIR1 : 1; // 9
			uint32_t SPIENDIR : 1; // 10
			uint32_t SSB : 1; // 11
			uint32_t rsvd12 : 20; // 31:12
		} SYST_bit;
	}; // 0x124

	/* SYS_MCSPI_MODULCTRL register bit field */
	union {
		volatile uint32_t MODULCTRL;

		volatile struct {
			uint32_t SINGLE : 1; // 0
			uint32_t PIN34 : 1; // 1
			uint32_t MS : 1; // 2
			uint32_t SYSTEM_TEST : 1; // 3
			uint32_t INITDLY : 3; // 6:4
			uint32_t MOA : 1; // 7
			uint32_t FDAA : 1; // 8
			uint32_t rsvd9 : 23; // 31:9
		} MODULCTRL_bit;
	}; // 0x128

	/* SYS_MCSPI_CH0CONF register bit field */
	union {
		volatile uint32_t CH0CONF;

		volatile struct {
			uint32_t PHA : 1; // 0
			uint32_t POL : 1; // 1
			uint32_t CLKD : 4; // 5:2
			uint32_t EPOL : 1; // 6
			uint32_t WL : 5; // 11:7
			uint32_t TRM : 2; // 13:12
			uint32_t DMAW : 1; // 14
			uint32_t DMAR : 1; // 15
			uint32_t DPE0 : 1; // 16
			uint32_t DPE1 : 1; // 17
			uint32_t IS : 1; // 18
			uint32_t TURBO : 1; // 19
			uint32_t FORCE : 1; // 20
			uint32_t SPIENSLV : 2; // 22:21
			uint32_t SBE : 1; // 23
			uint32_t SBPOL : 1; // 24
			uint32_t TCS0 : 2; // 26:25
			uint32_t FFEW : 1; // 27
			uint32_t FFER : 1; // 28
			uint32_t CLKG : 1; // 29
			uint32_t rsvd30 : 2; // 31:30
		} CH0CONF_bit;
	}; // 0x12c

	/* SYS_MCSPI_CH0STAT register bit field */
	union {
		volatile uint32_t CH0STAT;

		volatile struct {
			uint32_t RXS : 1; // 0
			uint32_t TXS : 1; // 1
			uint32_t EOT : 1; // 2
			uint32_t TXFFE : 1; // 3
			uint32_t TXFFF : 1; // 4
			uint32_t RXFFE : 1; // 5
			uint32_t RXFFF : 1; // 6
			uint32_t rsvd7 : 25; // 31:7
		} CH0STAT_bit;
	}; // 0x130

	/* SYS_MCSPI_CH0CTRL register bit field */
	union {
		volatile uint32_t CH0CTRL;

		volatile struct {
			uint32_t EN : 1; // 0
			uint32_t rsvd1 : 7; // 7:1
			uint32_t EXTCLK : 8; // 15:8
			uint32_t rsvd16 : 16; // 31:16
		} CH0CTRL_bit;
	}; // 0x134

	/* SYS_MCSPI_TX0 register bit field */
	union {
		volatile uint32_t TX0;

		volatile struct {
			uint32_t TDATA : 32; // 31:0
		} TX0_bit;
	}; // 0x138

	/* SYS_MCSPI_RX0 register bit field */
	union {
		volatile uint32_t RX0;

		volatile struct {
			uint32_t RDATA : 32; // 31:0
		} RX0_bit;
	}; // 0x13c

	/* SYS_MCSPI_CH1CONF register bit field */
	union {
		volatile uint32_t CH1CONF;

		volatile struct {
			uint32_t PHA : 1; // 0
			uint32_t POL : 1; // 1
			uint32_t CLKD : 4; // 5:2
			uint32_t EPOL : 1; // 6
			uint32_t WL : 5; // 11:7
			uint32_t TRM : 2; // 13:12
			uint32_t DMAW : 1; // 14
			uint32_t DMAR : 1; // 15
			uint32_t DPE0 : 1; // 16
			uint32_t DPE1 : 1; // 17
			uint32_t IS : 1; // 18
			uint32_t TURBO : 1; // 19
			uint32_t FORCE : 1; // 20
			uint32_t rsvd21 : 2; // 22:21
			uint32_t SBE : 1; // 23
			uint32_t SBPOL : 1; // 24
			uint32_t TCS1 : 2; // 26:25
			uint32_t FFEW : 1; // 27
			uint32_t FFER : 1; // 28
			uint32_t CLKG : 1; // 29
			uint32_t rsvd30 : 2; // 31:30
		} CH1CONF_bit;
	}; // 0x140

	/* SYS_MCSPI_CH1STAT register bit field */
	union {
		volatile uint32_t CH1STAT;

		volatile struct {
			uint32_t RXS : 1; // 0
			uint32_t TXS : 1; // 1
			uint32_t EOT : 1; // 2
			uint32_t TXFFE : 1; // 3
			uint32_t TXFFF : 1; // 4
			uint32_t RXFFE : 1; // 5
			uint32_t RXFFF : 1; // 6
			uint32_t rsvd7 : 25; // 31:7
		} CH1STAT_bit;
	}; // 0x144

	/* SYS_MCSPI_CH1CTRL register bit field */
	union {
		volatile uint32_t CH1CTRL;

		volatile struct {
			uint32_t EN : 1; // 0
			uint32_t rsvd1 : 7; // 7:1
			uint32_t EXTCLK : 8; // 15:8
			uint32_t rsvd16 : 16; // 31:16
		} CH1CTRL_bit;
	}; // 0x148

	/* SYS_MCSPI_TX1 register bit field */
	union {
		volatile uint32_t TX1;

		volatile struct {
			uint32_t TDATA : 32; // 31:0
		} TX1_bit;
	}; // 0x14c

	/* SYS_MCSPI_RX1 register bit field */
	union {
		volatile uint32_t RX1;

		volatile struct {
			uint32_t RDATA : 32; // 31:0
		} RX1_bit;
	}; // 0x150

	/* SYS_MCSPI_CH2CONF register bit field */
	union {
		volatile uint32_t CH2CONF;

		volatile struct {
			uint32_t PHA : 1; // 0
			uint32_t POL : 1; // 1
			uint32_t CLKD : 4; // 5:2
			uint32_t EPOL : 1; // 6
			uint32_t WL : 5; // 11:7
			uint32_t TRM : 2; // 13:12
			uint32_t DMAW : 1; // 14
			uint32_t DMAR : 1; // 15
			uint32_t DPE0 : 1; // 16
			uint32_t DPE1 : 1; // 17
			uint32_t IS : 1; // 18
			uint32_t TURBO : 1; // 19
			uint32_t FORCE : 1; // 20
			uint32_t rsvd21 : 2; // 22:21
			uint32_t SBE : 1; // 23
			uint32_t SBPOL : 1; // 24
			uint32_t TCS2 : 2; // 26:25
			uint32_t FFEW : 1; // 27
			uint32_t FFER : 1; // 28
			uint32_t CLKG : 1; // 29
			uint32_t rsvd30 : 2; // 31:30
		} CH2CONF_bit;
	}; // 0x154

	/* SYS_MCSPI_CH2STAT register bit field */
	union {
		volatile uint32_t CH2STAT;

		volatile struct {
			uint32_t RXS : 1; // 0
			uint32_t TXS : 1; // 1
			uint32_t EOT : 1; // 2
			uint32_t TXFFE : 1; // 3
			uint32_t TXFFF : 1; // 4
			uint32_t RXFFE : 1; // 5
			uint32_t RXFFF : 1; // 6
			uint32_t rsvd7 : 25; // 31:7
		} CH2STAT_bit;
	}; // 0x158

	/* SYS_MCSPI_CH2CTRL register bit field */
	union {
		volatile uint32_t CH2CTRL;

		volatile struct {
			uint32_t EN : 1; // 0
			uint32_t rsvd1 : 7; // 7:1
			uint32_t EXTCLK : 8; // 15:8
			uint32_t rsvd16 : 16; // 31:16
		} CH2CTRL_bit;
	}; // 0x15c

	/* SYS_MCSPI_TX2 register bit field */
	union {
		volatile uint32_t TX2;

		volatile struct {
			uint32_t TDATA : 32; // 31:0
		} TX2_bit;
	}; // 0x160

	/* SYS_MCSPI_RX2 register bit field */
	union {
		volatile uint32_t RX2;

		volatile struct {
			uint32_t RDATA : 32; // 31:0
		} RX2_bit;
	}; // 0x164

	/* SYS_MCSPI_CH3CONF register bit field */
	union {
		volatile uint32_t CH3CONF;

		volatile struct {
			uint32_t PHA : 1; // 0
			uint32_t POL : 1; // 1
			uint32_t CLKD : 4; // 5:2
			uint32_t EPOL : 1; // 6
			uint32_t WL : 5; // 11:7
			uint32_t TRM : 2; // 13:12
			uint32_t DMAW : 1; // 14
			uint32_t DMAR : 1; // 15
			uint32_t DPE0 : 1; // 16
			uint32_t DPE1 : 1; // 17
			uint32_t IS : 1; // 18
			uint32_t TURBO : 1; // 19
			uint32_t FORCE : 1; // 20
			uint32_t rsvd21 : 2; // 22:21
			uint32_t SBE : 1; // 23
			uint32_t SBPOL : 1; // 24
			uint32_t TCS3 : 2; // 26:25
			uint32_t FFEW : 1; // 27
			uint32_t FFER : 1; // 28
			uint32_t CLKG : 1; // 29
			uint32_t rsvd30 : 2; // 31:30
		} CH3CONF_bit;
	}; // 0x168

	/* SYS_MCSPI_CH3STAT register bit field */
	union {
		volatile uint32_t CH3STAT;

		volatile struct {
			uint32_t RXS : 1; // 0
			uint32_t TXS : 1; // 1
			uint32_t EOT : 1; // 2
			uint32_t TXFFE : 1; // 3
			uint32_t TXFFF : 1; // 4
			uint32_t RXFFE : 1; // 5
			uint32_t RXFFF : 1; // 6
			uint32_t rsvd7 : 25; // 31:7
		} CH3STAT_bit;
	}; // 0x16c

	/* SYS_MCSPI_CH3CTRL register bit field */
	union {
		volatile uint32_t CH3CTRL;

		volatile struct {
			uint32_t EN : 1; // 0
			uint32_t rsvd1 : 7; // 7:1
			uint32_t EXTCLK : 8; // 15:8
			uint32_t rsvd16 : 16; // 31:16
		} CH3CTRL_bit;
	}; // 0x170

	/* SYS_MCSPI_TX3 register bit field */
	union {
		volatile uint32_t TX3;

		volatile struct {
			uint32_t TDATA : 32; // 31:0
		} TX3_bit;
	}; // 0x174

	/* SYS_MCSPI_RX3 register bit field */
	union {
		volatile uint32_t RX3;

		volatile struct {
			uint32_t RDATA : 32; // 31:0
		} RX3_bit;
	}; // 0x178

	/* SYS_MCSPI_XFERLEVEL register bit field */
	union {
		volatile uint32_t XFERLEVEL;

		volatile struct {
			uint32_t AEL : 8; // 7:0
			uint32_t AFL : 8; // 15:8
			uint32_t WCNT : 16; // 31:16
		} XFERLEVEL_bit;
	}; // 0x17c

	/* SYS_MCSPI_DAFTX register bit field */
	union {
		volatile uint32_t DAFTX;

		volatile struct {
			uint32_t DAFTDATA : 32; // 31:0
		} DAFTX_bit;
	}; // 0x180

	uint8_t rsvd184[28]; // 0x184 - 0x19f

	/* SYS_MCSPI_DAFRX register bit field */
	union {
		volatile uint32_t DAFRX;

		volatile struct {
			uint32_t DAFRDATA : 32; // 31:0
		} DAFRX_bit;
	}; // 0x1a0

} sysMcspi;

#define MCSPI1 (*((volatile sysMcspi*)0x48098000))
#define MCSPI2 (*((volatile sysMcspi*)0x4809A000))
#define MCSPI3 (*((volatile sysMcspi*)0x480B8000))
#define MCSPI4 (*((volatile sysMcspi*)0x480BA000))

#endif /* _SYS_MCSPI_H_ */
