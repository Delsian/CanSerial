/*
 * Main starting point for CanSerial - virtual serial ports for CAN devices
 *
 *  Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
 *  This file may be distributed under the terms of the GNU GPLv3 license.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>

#include "cansock.h"
#include "portnumber.h"

static int running;

static void cleanup_handler(int signo)
{
	if (signo == SIGINT) {
		printf("Received SIGINT\n");
		running = 0;

		CanSockClose();
	}
}

int main(int argc, char **argv)
{
	int retval;
	int i;

	if(signal(SIGINT, cleanup_handler) == SIG_ERR) {
		fprintf(stderr, "Can't catch SIGINT\n");
	} else {
		running = 1;
	}

	PnInit();

	if ( (retval = CanSockInit()) != 0) {
		fprintf(stderr, "Socket init error: %d\n", retval);

		return 1;
	}

	/* send frame */
	while(running)
	{
		CanPing();
		usleep(3000000); /* Delay 3s before next loop */
	}
	return 0;
}

