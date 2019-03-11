#ifndef CANSOCK_H_
#define CANSOCK_H_

#include <linux/can.h>

// Read UUID  (6bytes)
#define PKT_ID_UUID (0x321)
// Set address (2bytes) to UUID (6b)
#define PKT_ID_SET (0x322)
// UUID response from slave  (6bytes)
#define PKT_ID_UUID_RESP (0x323)
#define PKT_ID_UUID_FILTER (0x320)
#define PKT_ID_UUID_MASK (0xFFFC)
// ID's starts from this number
#define PKT_ID_CTL_FILTER (0x180)
// And total 127 ports max
#define PKT_ID_CTL_MASK (0xFF80)
#define NUM_CAN_FILTERS 2

#define CAN_DATA_SIZE (8)

#define PINGS_BEFORE_DISCONNECT 4

typedef struct can_frame tCanFrame;

typedef struct {
	uint16_t port;
	canid_t canid;
	int pingcount;
} tPortId;

// p[0] unused and VportFd[0] means CAN socket
typedef struct {
	tPortId *p;
	struct pollfd *VportFd;
	int portsize;
	int portptr;
} tPorts;


int CanSockInit(void);
void CanSockClose(void);
int CanSockSend(canid_t id, uint8_t len, uint8_t* data);
void CanPing(void);

#endif /* CANSOCK_H_ */
