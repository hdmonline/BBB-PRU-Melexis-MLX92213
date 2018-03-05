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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/poll.h>
#include <stdlib.h>
#include <time.h>
#include "MQTTClient.h"

/* rpmsg constants */
#define MAX_BUFFER_SIZE		512
float readBuf[20];
#define DEVICE_NAME		"/dev/rpmsg_pru31"
char payload[200];

/* MQTT constants */
#define ADDRESS     "mb12.iotfm.org:1883"
#define CLIENTID    "GenosSensor"
#define TOPIC       "Asset/GENOS1234567/SpindleSpeed2"
#define QOS         1
#define TIMEOUT		10000L
#define ASSETID    	"GENOS1234567"
#define DATAITEMID	"SpindleSpeed2"


/* Time parameters and constants */
volatile clock_t last_time;
const int pub_interval = 1000; // ms

/* average rpm */
volatile float ave_rpm = 0;
volatile int i_sample = 0;

/* MQTT functions */
volatile MQTTClient_deliveryToken deliveredtoken;
void delivered(void *context, MQTTClient_deliveryToken dt)
{
	printf("Message with token value %d delivery confirmed\n", dt);
	deliveredtoken = dt;
}
int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
	int i;
	char* payloadptr;
	printf("Message arrived\n");
	printf("     topic: %s\n", topicName);
	printf("   message: ");
	payloadptr = message->payload;
	for(i=0; i<message->payloadlen; i++)
	{
		putchar(*payloadptr++);
	}
	putchar('\n');
	MQTTClient_freeMessage(&message);
	MQTTClient_free(topicName);
	return 1;
}
void connlost(void *context, char *cause)
{
	printf("\nConnection lost\n");
	printf("cause: %s\n", cause);
}

/* elapsed function */
int isElapsed(void)
{
	// Stroing start time
	clock_t curr_time = clock();

	// looping till required time is not acheived
	int rt = (curr_time - last_time > pub_interval) ? 1 : 0;
	return rt;
}

int main(void)
{
	/* create MQTT client */
	MQTTClient client;
	MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
	MQTTClient_message pubmsg = MQTTClient_message_initializer;
	MQTTClient_deliveryToken token;
	int rc;
	MQTTClient_create(&client, ADDRESS, CLIENTID,
					  MQTTCLIENT_PERSISTENCE_NONE, NULL);

	/* create MQTT connection */
	conn_opts.keepAliveInterval = 20;
	conn_opts.cleansession = 1;
	conn_opts.username = "mb12dongmin";
	conn_opts.password = "M5spKnCW3ZMAZsSH";
	MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
	if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
	{
		printf("Failed to connect, return code %d\n", rc);
		exit(EXIT_FAILURE);
	}

	pubmsg.qos = QOS;
	pubmsg.retained = 0;
	deliveredtoken = 0;


	struct pollfd pollfds[1];
	int result = 0;

	/* Open the rpmsg_pru character device file */
	pollfds[0].fd = open(DEVICE_NAME, O_RDWR);

	/*
	 * If the RPMsg channel doesn't exist yet the character device
	 * won't either.
	 * Make sure the PRU firmware is loaded and that the rpmsg_pru
	 * module is inserted.
	 */
	if (pollfds[0].fd < 0) {
		printf("Failed to open %s\n", DEVICE_NAME);
		return -1;
	}

	/* The RPMsg channel exists and the character device is opened */
	printf("Opened %s, sending messages\n\n", DEVICE_NAME);

	result = write(pollfds[0].fd, "hello PRU!", 10);
	if (result > 0)
		printf("Message Sent to PRU\n");

    last_time = clock();

	while(1){

		/* Poll until we receive a message from the PRU and then print it */
		result = read(pollfds[0].fd, readBuf, 1);

		if (result > 0) {
            if (isElapsed() == 1) {
                ave_rpm = (ave_rpm*i_sample+readBuf[0])/(i_sample+1);
                printf("rpm:%.2f\n\n", ave_rpm);
                sprintf(payload, "{\"assetId\":\"%s\",\"dataItemId\":\"%s\",\"value\":\"%.2f\"}", ASSETID, DATAITEMID, ave_rpm);
                pubmsg.payload = payload;
                pubmsg.payloadlen = strlen(payload);
                MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
                while(deliveredtoken != token);
                last_time = clock();
                i_sample = 0;
                ave_rpm= 0;
            } else {
                ave_rpm = (ave_rpm*i_sample+readBuf[0])/(i_sample+1);
                i_sample++;
            }

		}
	}

	/* Received all the messages the example is complete */
	printf("Received messages, closing %s\n", DEVICE_NAME);

	/* Close the rpmsg_pru character device file */
	close(pollfds[0].fd);

	return 0;
}
