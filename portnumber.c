/*
 * Port number allocation for CanSerial
 *
 *  Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
 *  This file may be distributed under the terms of the GNU GPLv3 license.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "cansock.h"
#include "portnumber.h"

#define CONFIG_LINE_BUFFER_SIZE 64
#define CONFIG_FILENAME "/var/tmp/canuuids.cfg"

typedef struct {
	uint16_t port;
	uint8_t uuid[PORT_UUID_SIZE];
} tPnKeep;

static int pn_len = 0;
static int max_pn = 0;
static tPnKeep *dict;

// power of two
static int dictsize = 2;

static void printuuid(FILE* stream, uint8_t *u)
{
	int i;
	for (i=0; i< PORT_UUID_SIZE; i++) {
		if (i>0)
			fprintf(stream, ":");
		fprintf(stream, "%02X", u[i]);
	}
}

static void addnum(uint16_t p, uint8_t *u)
{
	if(pn_len+1 >= dictsize) {
		// Increase allocated space
		dictsize *= 2;
		dict = realloc(dict, sizeof(tPnKeep) * dictsize);
		if (!dict) {
			fprintf(stderr, "realloc failed!\n");
			exit(1);
		}
	}

	// Check duplicates
	for (int i=0; i<pn_len; i++) {
		if (dict[i].port == p ||
				memcmp(dict[i].uuid, u, PORT_UUID_SIZE) == 0) {
			printf("Duplicate port %d\n", p);
			return;
		}
	}

	memcpy(dict[pn_len].uuid, u, PORT_UUID_SIZE);
	dict[pn_len].port = p;
	//printf("port %d ", p);
	//printuuid(stdout, u);
	//printf("\n");
	pn_len++;

	// max port for automatic allocation
	if(max_pn<p) max_pn = p;
}

void PnInit(void)
{
	FILE *fp;
	char buf[CONFIG_LINE_BUFFER_SIZE];

	dict = malloc(sizeof(tPnKeep) * dictsize);
	if (!dict) {
		fprintf(stderr, "malloc failed!\n");
		exit(1);
	}

    if (fp=fopen(CONFIG_FILENAME, "r")) {
        while(fgets(buf, CONFIG_LINE_BUFFER_SIZE, fp) > 0) {
            if (buf[0] == '#' || strlen(buf) < 4) {
                continue;
            }
            int p = atoi(buf);
            char c[10];
            uint8_t u[PORT_UUID_SIZE] = {0};
            if (sscanf(buf, "%s %hhx:%hhx:%hhx:%hhx:%hhx:%hhx\n",
                    c, &(u[0]), &(u[1]), &(u[2]), &(u[3]), &(u[4]), &(u[5])) == 7) {
                addnum(p,u);
            } else {
                printf("Wrong format %s\n", buf);
            }
        }
        fclose(fp);
    } else {
        // First run
        if ((fp=fopen(CONFIG_FILENAME, "w")) != NULL) {
            fprintf(fp,"# [port] [UUID]\n");
            fclose(fp);
        } else {
	    perror(CONFIG_FILENAME);
	}
    }
}

void PnStore(int port, uint8_t* u)
{
	FILE *fp;

	if ((fp=fopen(CONFIG_FILENAME, "a")) != NULL) {
		fprintf(fp,"%d ",port);
		printuuid(fp, u);
		fprintf(fp,"\n");
		fclose(fp);
	} else {
		perror("Can't store config " CONFIG_FILENAME);
	}
}

uint16_t PnGetNumber(uint8_t* u)
{
	for (int i=0; i<pn_len; i++) {
		if (memcmp(dict[i].uuid, u, PORT_UUID_SIZE) == 0) {
			return dict[i].port;
		}
	}
	// not found, keep num in dict
	addnum(max_pn+1,u);
	printf("Address ");
	printuuid(stdout, u);
	printf(" not found in config, assigned port %d\n", max_pn);
	PnStore(max_pn, u);

	return max_pn; // already incremented value in previous call
}


