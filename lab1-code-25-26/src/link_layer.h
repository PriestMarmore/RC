// Link layer header.
// DO NOT CHANGE THIS FILE

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

#include <sys/types.h>
#include <termios.h>


typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// Size of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer.
#define MAX_PAYLOAD_SIZE 1000

// Frame constants (useful for sizing buffers)
#define SU_FRAME_SIZE 5

// MISC
#define FALSE 0
#define TRUE 1

// Open a connection using the "port" parameters defined in struct linkLayer.
// Return 0 on success or -1 on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or -1 on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or -1 on error.
int llread(unsigned char *packet);

// Close previously opened connection and print transmission statistics in the console.
// Return 0 on success or -1 on error.
int llclose();

// State Machine Prototype (for reading control frames)
int read_su_frame(int fd, unsigned char *buffer);

// Alarm/Signal handler functions (M4)
void setupAlarmHandler();
void enableAlarm(int timeoutSec, int maxTries);
void disableAlarm();
int isAlarmSet();
void clearAlarm();
int getRetransmissionCount();
int getMaxRetransmissionCount();
void resetAlarm();

#endif // _LINK_LAYER_H_
