/*
 * Source Modified by Pierrick Rauby <PierrickRauby - pierrick.rauby@gmail.com>
 * Based on the examples distributed by TI
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdint.h>
#include <stdio.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_iep.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include <stdlib.h>
#include <string.h>
#include "resource_table_1.h"

/* Register r30 and r31 */
volatile register uint32_t __R30;
volatile register uint32_t __R31;

uint8_t payload[RPMSG_BUF_SIZE];

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT		((uint32_t) 1 << 31)
/* CMP0 and CMP1 sets bit 0 and 1 in register TMR_CMP_STS */
#define CMP0_STS		((uint32_t) 1)
#define CMP1_STS		((uint32_t) 1 << 1)
/* input <---> pru1 r30/31.0 */
#define P8_45			((uint32_t) 1)
/* output (enable) <---> pru1 r30/31.1 */
#define P8_46			((uint32_t) 1 << 1)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			18
#define FROM_ARM_HOST			19

/*
 * Using the name 'rpmsg-client-sample' will probe the RPMsg sample driver
 * found at linux-x.y.z/samples/rpmsg/rpmsg_client_sample.c
 */
//#define CHAN_NAME			"rpmsg-client-sample"
#define CHAN_NAME			"rpmsg-pru"

#define CHAN_DESC			"Channel 31"
#define CHAN_PORT			31

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

/* enable signal frequency */
#define EN_FREQ			((uint32_t) 10000)
/* enable signal width (us) */
#define EN_WIDTH		((uint32_t) 10)
/* pru clock freq */
#define PRU_CLK			((uint32_t) 200000000)


// Enable signal period: compare register 0
uint32_t CMP0 = PRU_CLK/EN_FREQ;
// Enable signal width: compare register 1
uint32_t CMP1 = PRU_CLK/1000000*EN_WIDTH;

/* number of cycles before next rising edge */
volatile uint32_t iCycle = 0;

/* Time cycles */
volatile uint32_t count = 0;
volatile uint32_t last_count = 0;
volatile uint32_t interval = 0;
/* Get initial state */
volatile uint32_t last_input = 0;
volatile uint32_t curr_input = 0;



/* Put float rpm and uint8 payload in a union type */
union rpm_union{
    float rpm;
    uint8_t payload[4];
}rpm_payload;

void init_iep(void) {
	// Set counter to 0
	CT_IEP.TMR_CNT = 0x0;
	// Enable CMP0 and CMP1
	CT_IEP.TMR_CMP_CFG = 0x07;
	// Set CMP registers
	CT_IEP.TMR_CMP0 = CMP0;
	CT_IEP.TMR_CMP1 = CMP1;
	// Set output bit to high
	__R30 |= P8_46;
	// Enable counter, set default incremental to 1
	CT_IEP.TMR_GLB_CFG = 0x11;
}

//void reset_iep(void) {
//	// Set counter to 0
//	CT_IEP.TMR_CNT = 0x0;
//	// Enable counter, set incremental to 1 (default 5)
//	CT_IEP.TMR_GLB_CFG = 0x11;
//}

/*
 * main.c
 */
void main(void)
{
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	volatile uint8_t *status;

	/* Allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* reset the ied and start the counter */
	init_iep();

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    /* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);

    /* Wait for the dummy message from host */
    while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) != PRU_RPMSG_SUCCESS);

	while (1) {
		/* Check bit 0 of register R31 to see if the input changes */
		curr_input = __R31 & P8_45;

		/* event cmp1, set output bit to low */
		if (CT_IEP.TMR_CMP_STS & CMP1_STS){
			/* clear event */
			CT_IEP.TMR_CMP_STS |= CMP1_STS;

			/* Set output bit to low */
			__R30 &= ~P8_46;
		}

		/* event cmp0, set output bit to high */
		if (CT_IEP.TMR_CMP_STS & CMP0_STS){
			iCycle++;
			/* clear event */
			CT_IEP.TMR_CMP_STS |= CMP0_STS;
			CT_IEP.TMR_CMP_STS |= CMP1_STS;
			/* Set output bit to high */
			__R30 |= P8_46;
		}

		/* Polling on the input to catch the rising edge */
		if (curr_input != last_input) {

            if (curr_input == 1){
                /* Send the counter value of the time back to ARM */
                count = CT_IEP.TMR_CNT;

				/* Calculate the number of clock cycles between 2 rising edges */
				interval = (last_count > count) ? last_count-count+iCycle*CMP0 : (iCycle-1)*CMP0+last_count-count;

                /* Calculate the rpm */
                rpm_payload.rpm = (float)PRU_CLK/(float)interval*60;

				/* Send rpm to host */
                pru_rpmsg_send(&transport, dst, src, rpm_payload.payload, 4);

				/* reset the number of cycles */
				iCycle = 0;

				last_count = count;
            }

            last_input = curr_input;
		}
	}
}
