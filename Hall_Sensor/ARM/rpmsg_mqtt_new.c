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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include "MQTTClient.h"

/* rpmsg constants */
#define MAX_BUFFER_SIZE		512
float readBuf[20];
#define DEVICE_NAME		"/dev/rpmsg_pru31"
char payload[200];

/* MQTT constants */
#define ADDRESS     "mb12.iotfm.org:1883"
#define CLIENTID    "GenosSensor"
#define TOPIC       "Asset/GENOSD2814/SpindleSpeed2"
#define QOS         1
#define TIMEOUT		10000L
#define ASSETID    	"GENOSD2814"
#define DATAITEMID	"SpindleSpeed2"


/* Time parameters and constants */
volatile clock_t last_time;
clock_t time_stamp;
const long pub_interval = 3000; // clock cycles
char str_clock_buf[20];

/* average rpm */
volatile float ave_rpm = 0;
volatile int i_sample = 0;
volatile int isZero = 0;

/* MQTT functions */
volatile MQTTClient_deliveryToken deliveredtoken;
void delivered(void *context, MQTTClient_deliveryToken dt)
{
	printf("Message with token value %d delivery confirmed\n\n", dt);
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
long isElapsed(void)
{
	// Stroing start time
	clock_t curr_time = clock();
	long time_elapsed = (long)curr_time - (long)last_time;
	// looping till required time is not acheived
	int rt = (time_elapsed > pub_interval) ? 1 : 0;
	if (rt == 1){
		printf("time elaspsed: %d\n", time_elapsed);
	}
	return rt;
}

/* if it hasn't get any interrupt from PRU for a long time, send 0 */
long isElapsed2(void)
{
	// Stroing start time
	clock_t curr_time = clock();
	long time_elapsed = (long)curr_time - (long)last_time;
	// looping till required time is not acheived
	int rt = (time_elapsed > (CLOCKS_PER_SEC/5)) ? 1 : 0;
	if (rt == 1){
		printf("time elaspsed: %d\n", time_elapsed);
	}
	return rt;
}

int main(void)
{
	fd_set set;
	struct timeval timeout;
	int rv;

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




	///////////
	time_stamp = time(NULL);
	strftime(str_clock_buf, 20, "%Y-%m-%dT%H:%M:%S", localtime(&time_stamp));
	printf("time: %s\n", str_clock_buf);
	///////////


//	struct pollfd pollfds[1];
//	int result = 0;

	/* Open the rpmsg_pru character device file */
//	pollfds[0].fd = open(DEVICE_NAME, O_RDWR);

	int filedesc = open(DEVICE_NAME, O_RDWR);
	FD_ZERO(&set);
	FD_SET(filedesc, &set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 5000;


	/*
	 * If the RPMsg channel doesn't exist yet the character device
	 * won't either.
	 * Make sure the PRU firmware is loaded and that the rpmsg_pru
	 * module is inserted.
	 */
//	if (pollfds[0].fd < 0) {
//		printf("Failed to open %s\n", DEVICE_NAME);
//		return -1;
//	}

	/* The RPMsg channel exists and the character device is opened */
	printf("Opened %s, sending messages\n\n", DEVICE_NAME);

	int result = 0;
	result = write(filedesc, "hello PRU!", 10);
	if (result > 0)
		printf("Message Sent to PRU\n");

    last_time = clock();

	while(1){

		/* Poll until we receive a message from the PRU and then print it */
		rv = select(filedesc + 1, &set, NULL, NULL, &timeout);
		if (rv == 1) {
			perror("select");
		}else if (rv == 0){
			printf("sending 0");
			time_stamp = time(NULL);
			strftime(str_clock_buf, 20, "%Y-%m-%dT%H:%M:%S", localtime(&time_stamp));
			sprintf(payload, "{\"assetId\":\"%s\",\"dateTime\":\"%s\",\"dataItemId\":\"%s\",\"value\":\"%.2f\"}",
					ASSETID, str_clock_buf, DATAITEMID, 0.00);
			pubmsg.payload = payload;
			pubmsg.payloadlen = strlen(payload);
			MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
			last_time = clock();
			i_sample = 0;
			ave_rpm = 0;
			isZero = 1;
		}else{
			int result = read(filedesc, readBuf, 1);
			if (result > 0) {
				isZero = 0;
				if (isElapsed() == 1) {
					ave_rpm = (ave_rpm*i_sample+readBuf[0])/(i_sample+1);
					printf("rpm:%.2f\n", ave_rpm);
					time_stamp = time(NULL);
					strftime(str_clock_buf, 20, "%Y-%m-%dT%H:%M:%S", localtime(&time_stamp));
					sprintf(payload, "{\"assetId\":\"%s\",\"dateTime\":\"%s\",\"dataItemId\":\"%s\",\"value\":\"%.2f\"}", ASSETID, str_clock_buf, DATAITEMID, ave_rpm);

					//printf("time: %s\n", str_clock_buf);

					pubmsg.payload = payload;
					pubmsg.payloadlen = strlen(payload);
					MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
					while(deliveredtoken != token);
					last_time = clock();
					i_sample = 0;
					ave_rpm= 0;
				}else {
					ave_rpm = (ave_rpm*i_sample+readBuf[0])/(i_sample+1);
					i_sample++;
				}
			}
		}
	}

	/* Received all the messages the example is complete */
	printf("Received messages, closing %s\n", DEVICE_NAME);

	/* Close the rpmsg_pru character device file */
	//close(pollfds[0].fd);

	return 0;
}
