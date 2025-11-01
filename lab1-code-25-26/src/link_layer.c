// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// Others
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// --- Role Definitions ---
#define TX 0
#define RX 1

// Frame constants (BCC1 = A ^ C)
#define FLAG 0x7E
#define A_TX 0x03   // Address field for Transmitter (Commands) / Receiver (Replies)
#define A_RX 0x01   // Address field for Receiver (Commands) / Transmitter (Replies)
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define SU_FRAME_SIZE 5 // Size of SET, UA, DISC frames

// Control field values for I-frames (Ns=0 or Ns=1)
#define C_I_0 0x00
#define C_I_1 0x80 

// Control field values for RR/REJ frames (Nr=0 or Nr=1)
// RR0: ready to receive I(0), Acknowledges I(1)
#define C_RR_0 0xAA 
// RR1: ready to receive I(1), Acknowledges I(0)
#define C_RR_1 0xAB 
// REJ0: rejects I(0)
#define C_REJ_0 0x54 
// REJ1: rejects I(1)
#define C_REJ_1 0x55 

// Stuffing constants
#define ESCAPE_BYTE 0x7D
#define XOR_BYTE 0x20

// -------------------------------------------------------

// --- Configuration Constants ---
#define MAX_ATTEMPTS 3
#define TIMEOUT 4

// --- Global Variable Definitions (To fix linker errors) ---
int g_role = 0; // Role of the application (TX or RX, 0 or 1). Will be set in llopen.
int g_alarm_flag = 0; // Flag set by the alarm signal handler.

// --- External Declarations (Functions and variables defined elsewhere) ---
extern int fd1;
extern void disableAlarm();
extern void clearAlarm();
// CORRECTED: Match your serial_port.h signature: returns int, takes const unsigned char*
extern int writeBytesSerialPort(const unsigned char *buf, int len); 
extern void create_su_frame(unsigned char *frame, unsigned char A, unsigned char C);
// REMOVED: extern int llwait_for_frame(...) to fix conflict with static definition.

// -------------------------------------------------------

// Global variables for connection state
int fd1 = -1; // File descriptor for the serial port
LinkLayer g_linkLayer; // Stores the connection parameters

// Sequence number for I-frames (Transmitter Ns, Receiver Nr)
unsigned char g_Ns = 0; // Next sequence number to SEND (0 or 1)
unsigned char g_Nr = 0; // Next sequence number EXPECTED (0 or 1)

// External functions defined in alarm_sigaction.c
extern void setupAlarmHandler();
extern void enableAlarm(int timeout, int maxRetransmissions); 
extern void disableAlarm();
extern int isAlarmSet();
extern void clearAlarm();
extern int getRetransmissionCount();

// Helper functions for M2/M4
void create_su_frame(unsigned char *frame, unsigned char A, unsigned char C) {
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = A ^ C; // BCC1 (A XOR C)
    frame[4] = FLAG;
}

int read_su_frame(int fd, unsigned char *buffer) {
    enum State {
        START,
        FLAG_RCV,
        A_RCV,
        C_RCV,
        BCC_RCV,
        STOP
    } state = START;

    unsigned char byte;
    int bytes_received = 0;

    // Loop until a full frame is received or read() times out (if VTIME > 0)
    while (state != STOP) {
        int res = read(fd, &byte, 1);
        
        if (res <= 0) {
            if (errno == EINTR) return 0;
            return 0;
        }

        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    buffer[bytes_received++] = byte;
                }
                break;
            case FLAG_RCV:
                if (byte == FLAG) {
                    bytes_received = 1; 
                } else if (byte == A_TX || byte == A_RX) {
                    state = A_RCV;
                    buffer[bytes_received++] = byte;
                } else {
                    state = START;
                    bytes_received = 0;
                }
                break;
            case A_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    bytes_received = 1;
                } 
                // Check for C_SET, C_UA, C_DISC, or RR/REJ
                else if (byte == C_SET || byte == C_UA || byte == C_DISC || byte == C_RR_0 || byte == C_RR_1 || byte == C_REJ_0 || byte == C_REJ_1) { 
                    state = C_RCV;
                    buffer[bytes_received++] = byte;
                } else {
                    state = START;
                    bytes_received = 0;
                }
                break;
            case C_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    bytes_received = 1;
                } 
                // BCC1 check: A XOR C
                else if (byte == (buffer[1] ^ buffer[2])) { 
                    state = BCC_RCV;
                    buffer[bytes_received++] = byte;
                } else {
                    state = START;
                    bytes_received = 0;
                }
                break;
            case BCC_RCV:
                if (byte == FLAG) {
                    buffer[bytes_received++] = byte;
                    state = STOP; // Valid frame found
                } else {
                    state = START;
                    bytes_received = 0;
                }
                break;
            case STOP:
                break;
        }

        if (bytes_received > SU_FRAME_SIZE) {
            state = START;
            bytes_received = 0;
        }
    }
    
    if (bytes_received == SU_FRAME_SIZE) {
        return bytes_received;
    }

    return 0;
}


// Helper functions for M3
unsigned char calculate_bcc2(const unsigned char *buffer, int bufferSize) {
    unsigned char bcc2 = 0x00;
    for (int i = 0; i < bufferSize; i++) {
        bcc2 ^= buffer[i];
    }
    return bcc2;
}

int byte_stuffing(const unsigned char *data, int dataSize, unsigned char *stuffedFrame) {
    int stuffedSize = 0;
    for (int i = 0; i < dataSize; i++) {
        if (data[i] == FLAG || data[i] == ESCAPE_BYTE) {
            stuffedFrame[stuffedSize++] = ESCAPE_BYTE;
            stuffedFrame[stuffedSize++] = data[i] ^ XOR_BYTE;
        } else {
            stuffedFrame[stuffedSize++] = data[i];
        }
    }
    return stuffedSize;
}

/**
 * @brief Performs byte de-stuffing on a buffer.
 * @param stuffedData The input buffer containing stuffed data.
 * @param stuffedSize The size of the stuffed input buffer.
 * @param destuffedData The output buffer to receive the de-stuffed bytes.
 * @return The number of de-stuffed bytes written to destuffedData.
 */
int byte_destuffing(const unsigned char *stuffedData, int stuffedSize, unsigned char *destuffedData) {
    int destuffedSize = 0;
    for (int i = 0; i < stuffedSize; i++) {
        if (stuffedData[i] == ESCAPE_BYTE) {
            // Check for potential buffer overflow before reading next byte
            if (i + 1 < stuffedSize) {
                destuffedData[destuffedSize++] = stuffedData[i+1] ^ XOR_BYTE;
                i++; // Skip the next byte as it was part of the stuffed sequence
            } else {
                // This indicates an incomplete stuffed sequence (ends with ESCAPE) - error
                fprintf(stderr, "Error: Incomplete stuffed sequence in frame.\n");
                return -1; 
            }
        } else {
            destuffedData[destuffedSize++] = stuffedData[i];
        }
    }
    return destuffedSize;
}

// Helper function to read a single supervision frame with timeout/retry logic
static int llwait_for_frame(unsigned char expected_A, unsigned char expected_C) {
    enum State {
        START,
        FLAG_RCV,
        A_RCV,
        C_RCV,
        BCC_RCV
    } state = START;

    unsigned char byte;
    unsigned char frame[SU_FRAME_SIZE];

    alarm(TIMEOUT);
    while (state != BCC_RCV) {
        // Read 1 byte at a time
        int res = read(fd1, &byte, 1);

        if (g_alarm_flag) {
            // Timeout occurred
            return 0; 
        }

        if (res <= 0) continue; // Skip if no byte read (but alarm flag should handle real timeouts)

        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frame[0] = byte;
                }
                break;
            case FLAG_RCV:
                if (byte == FLAG) {
                    frame[0] = byte; // Keep starting over if we see flags
                } else if (byte == expected_A) {
                    state = A_RCV;
                    frame[1] = byte;
                } else {
                    state = START;
                }
                break;
            case A_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frame[0] = byte;
                } else if (byte == expected_C) {
                    state = C_RCV;
                    frame[2] = byte;
                } else {
                    // Unexpected C frame (e.g., RR instead of UA). Restart.
                    state = START;
                }
                break;
            case C_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frame[0] = byte;
                } else if (byte == (frame[1] ^ frame[2])) { // Check BCC
                    state = BCC_RCV;
                    frame[3] = byte;
                } else {
                    // BCC mismatch. Frame corrupted. Restart.
                    state = START;
                }
                break;
            case BCC_RCV: // Should be unreachable
                break;
        }
    }
    alarm(0); // Stop alarm on successful frame read
    return 1; // Success
}

////////////////////////////////////////////////
// LLOPEN (M2 & M4 Implementation)
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    g_linkLayer = connectionParameters;

    // 1. OPEN AND CONFIGURE SERIAL PORT (M1)
    fd1 = openSerialPort(g_linkLayer.serialPort, g_linkLayer.baudRate);
    
    if (fd1 < 0) {
        fprintf(stderr, "ERROR: llopen failed to open serial port %s.\n", g_linkLayer.serialPort);
        return -1;
    }
    printf("Serial port %s opened successfully (fd=%d).\n", g_linkLayer.serialPort, fd1);


    if (connectionParameters.role == LlTx) {
        // --- TRANSMITTER LOGIC (M2 & M4) ---
        g_role = TX; // CRITICAL FIX: Set the global role
        
        // Setup the alarm signal handler
        setupAlarmHandler();
        
        const int maxAttempts = g_linkLayer.nRetransmissions;
        unsigned char set_frame[SU_FRAME_SIZE];
        unsigned char response_frame[SU_FRAME_SIZE];
        int bytes_read = 0;
        
        // Create the SET command frame: F, A_TX, C_SET, BCC1, F
        create_su_frame(set_frame, A_TX, C_SET);

        // Start the retransmission loop
        // Usa um contador explícito de tentativas
        for (int attempt = 1; attempt <= maxAttempts; ++attempt) {

            // 1. Envia SET
            printf("Tx: Sending SET frame... (attempt %d/%d)\n",
            attempt, maxAttempts);
            int written = writeBytesSerialPort(set_frame, SU_FRAME_SIZE);
            if (written < 0) {
                perror("Tx: writeBytesSerialPort failed");
                llclose();
                return -1;
            }

            // 2. Armamos o temporizador
            enableAlarm(g_linkLayer.timeout,maxAttempts);

            // 3. Espera por UA até timeout
            printf("Tx: Waiting for UA response...\n");
            while (!isAlarmSet()) {
                bytes_read = read_su_frame(fd1, response_frame);
                if (bytes_read == SU_FRAME_SIZE) {
                    if (response_frame[1] == A_TX && response_frame[2] == C_UA) {
                        printf("Tx: Received valid UA frame. Connection established.\n");
                        disableAlarm();
                        clearAlarm();
                        return fd1; // sucesso
                    }
                    else {
                        printf("Tx: Received frame, but not UA (A=0x%02x, C=0x%02x). Ignoring.\n",
                            response_frame[1], response_frame[2]);
                    }
                }
            }

            // Timeout desta tentativa
            disableAlarm();
            clearAlarm();
            printf("Tx: Timeout. Will retry if attempts left.\n");
        }

        // Esgotou as tentativas
        printf("Tx: Failed to establish connection after %d attempts\n",
            maxAttempts);
        llclose();
        return -1;
    }

    else if (connectionParameters.role == LlRx) {
        // --- RECEIVER LOGIC (M2) ---
        g_role = RX; // CRITICAL FIX: Set the global role
        
        // CRITICAL FIX: Setup the alarm handler for the RX role, so llwait_for_frame in llclose 
        // will not crash the process when SIGALRM is delivered.
        setupAlarmHandler(); 

        unsigned char received_frame[SU_FRAME_SIZE];
        int bytes_read = 0;
        
        printf("Rx: Waiting for SET frame...\n");

        // Loop indefinitely until a valid SET frame is received
        while (1) {
            bytes_read = read_su_frame(fd1, received_frame);

            if (bytes_read == SU_FRAME_SIZE) {
                // Check if the received frame is a valid SET (A=0x03, C=0x03)
                if (received_frame[1] == A_TX && received_frame[2] == C_SET) {
                    printf("Rx: Received valid SET frame. Sending UA...\n");
                    
                    // 1. Create UA response frame: F, A_TX, C_UA, BCC1, F
                    unsigned char ua_frame[SU_FRAME_SIZE];
                    create_su_frame(ua_frame, A_TX, C_UA); 
                    
                    // 2. Send UA response
                    if (writeBytesSerialPort(ua_frame, SU_FRAME_SIZE) < 0) {
                        perror("Rx: writeBytesSerialPort (UA) failed");
                        llclose();
                        return -1;
                    }
                    printf("Rx: Connection established.\n");
                    return fd1; // Success
                } else {
                    // Received a valid control frame, but not SET. Ignore and keep waiting.
                    printf("Rx: Received a frame, but not SET (A=0x%02x, C=0x%02x). Ignoring.\n", received_frame[1], received_frame[2]);
                }
            } else if (bytes_read < 0) {
                 perror("Rx: Error reading response frame");
                 llclose();
                 return -1;
            }
        }
    }
    
    return -1; 
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    // Max frame size: 5 (F,A,C,BCC1,F) + 2*(bufSize + 1 for BCC2)
    unsigned char stuffed_data[2 * bufSize + 1]; 
    unsigned char frame[2 * bufSize + 10]; 

    // Data for stuffing: payload + BCC2
    unsigned char *data_and_bcc2 = (unsigned char *)malloc(bufSize + 1);
    if (data_and_bcc2 == NULL) {
        perror("llwrite: malloc failed");
        return -1;
    }
    
    // Calculate BCC2 and prepare the data for stuffing
    unsigned char bcc2 = calculate_bcc2(buf, bufSize);
    memcpy(data_and_bcc2, buf, bufSize);
    data_and_bcc2[bufSize] = bcc2; 
    const int maxAttempts = g_linkLayer.nRetransmissions;


    // Start retransmission loop for the I-frame
    // Retransmissões com contador explícito (termina após N tentativas)
        for (int attempt = 1; attempt <= g_linkLayer.nRetransmissions; ++attempt) {
            // --- 1) Construção da trama I (a cada tentativa) ---
            int stuffedSize = byte_stuffing(data_and_bcc2, bufSize + 1, stuffed_data);

            frame[0] = FLAG;
            frame[1] = A_TX;
            frame[2] = (g_Ns == 0) ? C_I_0 : C_I_1;
            frame[3] = frame[1] ^ frame[2];
            memcpy(&frame[4], stuffed_data, stuffedSize);
            frame[4 + stuffedSize] = FLAG;
            int frameSize = 5 + stuffedSize;

            // --- 2) Envio ---
            printf("Tx: Sending I-frame (Ns=%d, dataSize=%d, frameSize=%d) [attempt %d/%d]\n",
                g_Ns, bufSize, frameSize, attempt, maxAttempts);

            int written = writeBytesSerialPort(frame, frameSize);
            if (written != frameSize) {
                perror("llwrite: Failed to write complete frame");
                free(data_and_bcc2);
                return -1;
            }

            // --- 3) Espera por RR/REJ com timeout ---
            enableAlarm(g_linkLayer.timeout, maxAttempts);
            printf("Tx: Waiting for RR/REJ response...\n");

            unsigned char response_frame[SU_FRAME_SIZE];
            int bytes_read = 0;

            while (!isAlarmSet()) {
                bytes_read = read_su_frame(fd1, response_frame);
                if (bytes_read == SU_FRAME_SIZE) {
                    unsigned char C_field = response_frame[2];
                    unsigned char expected_Nr = 1 - g_Ns;

                    // RR esperado?
                    if (C_field == ((expected_Nr == 0) ? C_RR_0 : C_RR_1)) {
                        printf("Tx: Received RR(%d). Frame acknowledged.\n", expected_Nr);
                        disableAlarm();
                        clearAlarm();
                        g_Ns = expected_Nr;      // alterna Ns
                        free(data_and_bcc2);
                        return bufSize;          // sucesso
                    }
                    // REJ do Ns atual?
                    if (C_field == ((g_Ns == 0) ? C_REJ_0 : C_REJ_1)) {
                        printf("Tx: Received REJ(%d). Retransmitting frame (Not counting as failure).\n", g_Ns);
                        disableAlarm();
                        clearAlarm();
                        // CRITICAL FIX: Decrement attempt counter. The next iteration of the 'for' loop
                        // will immediately increment it back up, thus ensuring REJ-driven retransmissions
                        // don't count towards the max failure limit.
                        attempt--; 
                        // Sai do while e volta ao for (nova tentativa)
                        break;
                    }

                    // Outros controlos: ignora e continua à espera até timeout
                }
            }

            if (isAlarmSet()) {
                printf("Tx: Timeout occurred. Retransmitting frame (attempt %d/%d).\n",
                    attempt, maxAttempts);
                disableAlarm();
                clearAlarm();
                // volta ao for para retransmitir
            }
        }

        // Esgotou tentativas
        printf("Tx: Failed to send frame after %d attempts (Max Retransmissions Reached).\n",
            g_linkLayer.nRetransmissions);
        free(data_and_bcc2);
        return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // The maximum possible destuffed size is limited by the packet buffer size
    // We assume the caller provides a buffer large enough for the maximum application layer packet
    // Assuming max frame size of 2 * 2000 (app layer packet max size) for safety
    unsigned char frame_buf[4096];
    unsigned char destuffed_payload_and_bcc2[4096];

    // State machine for reading the I-frame
    enum State {
        START,
        FLAG_RCV,
        A_RCV,
        C_RCV,
        BCC1_RCV,
        DATA_RCV,
        STOP
    } state = START;

    unsigned char byte;
    int frame_idx = 0;
    int data_start_idx = -1; // Index where the stuffed data/BCC2 starts

    printf("Rx: Waiting for I-frame (Ns=%d expected)...\n", g_Nr);

    while (state != STOP) {
        // We use a small buffer for safety, but read only 1 byte at a time for the state machine
        int res = read(fd1, &byte, 1);
        
        if (res <= 0) {
            // Error or Timeout: check if we were already in the middle of a frame.
            if (state != START) {
                 fprintf(stderr, "Rx: Timeout or Error while receiving frame in state %d. Restarting.\n", state);
            }
            // Reset to START and continue waiting
            state = START; 
            frame_idx = 0;
            data_start_idx = -1;
            
            // If the timeout was a read() timeout, return 0 to application layer
            if (errno == EINTR || res == 0) return 0; // Return 0 bytes delivered
            
            continue;
        }

        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frame_buf[frame_idx++] = byte;
                }
                break;
            case FLAG_RCV:
                if (byte == FLAG) {
                    frame_idx = 1; 
                } else if (byte == A_TX) {
                    state = A_RCV;
                    frame_buf[frame_idx++] = byte;
                } else {
                    state = START;
                    frame_idx = 0;
                }
                break;
            case A_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frame_idx = 1;
                }
                // Check if C is C_I_0 (Ns=0) or C_I_1 (Ns=1)
                else if (byte == C_I_0 || byte == C_I_1) { 
                    state = C_RCV;
                    frame_buf[frame_idx++] = byte;
                } else {
                    // Ignore other control frames (RR, REJ, DISC, etc.) and restart
                    state = START;
                    frame_idx = 0;
                }
                break;
            case C_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frame_idx = 1;
                }
                // Check BCC1: A XOR C
                else if (byte == (frame_buf[1] ^ frame_buf[2])) { 
                    frame_buf[frame_idx++] = byte;
                    data_start_idx = frame_idx; // Stuffed data starts here
                    state = DATA_RCV;
                } else {
                    // BCC1 error: Send REJ for the expected sequence number (g_Nr) and restart
                    fprintf(stderr, "Rx: BCC1 Error (0x%02x ^ 0x%02x != 0x%02x). Sending REJ(%d).\n", 
                            frame_buf[1], frame_buf[2], byte, g_Nr);
                    unsigned char rej_c = (g_Nr == 0) ? C_REJ_0 : C_REJ_1;
                    unsigned char rej_frame[SU_FRAME_SIZE];
                    create_su_frame(rej_frame, A_TX, rej_c);
                    writeBytesSerialPort(rej_frame, SU_FRAME_SIZE);
                    
                    state = START;
                    frame_idx = 0;
                    return 0; // RETURN 0: Frame discarded, application should retry llread.
                }
                break;
            case DATA_RCV:
                if (byte == FLAG) {
                    frame_buf[frame_idx++] = byte;
                    state = STOP; // Found end flag!
                } else {
                    frame_buf[frame_idx++] = byte;
                    // Check for buffer overflow
                    if (frame_idx >= 4096) {
                        fprintf(stderr, "Rx: Frame buffer overflow. Restarting.\n");
                        state = START;
                        frame_idx = 0;
                    }
                }
                break;
            case BCC1_RCV: // Not reachable now, but keeps the enum clean
            case STOP:
                break;
        }
    }

    // --- Frame validation and processing ---

    // 1. Get the control field Ns
    unsigned char received_Ns = (frame_buf[2] == C_I_1) ? 1 : 0;
    
    printf("Rx: Received I-frame (Ns=%d, Nr=%d expected).\n", received_Ns, g_Nr);

    // 2. Check Sequence Number (Ns must match Nr)
    if (received_Ns != g_Nr) {
        // Duplicate frame (Ns is the opposite of the current expected Nr, meaning we already received it)
        if (received_Ns == (1 - g_Nr)) {
            printf("Rx: Duplicate frame (Ns=%d). Sending RR(%d) for re-acknowledgement and discarding frame.\n", received_Ns, g_Nr);
            // Send RR(Nr) to acknowledge the frame with the sequence number we *do* expect next (the current g_Nr)
            unsigned char rr_c = (g_Nr == 0) ? C_RR_0 : C_RR_1;
            unsigned char rr_frame[SU_FRAME_SIZE];
            create_su_frame(rr_frame, A_TX, rr_c);
            writeBytesSerialPort(rr_frame, SU_FRAME_SIZE);
            return 0; // Discarded, application should retry llread.
        } else {
            // Unexpected Ns (Shouldn't happen in standard go-back-N, but safer to reject)
            fprintf(stderr, "Rx: Unexpected Ns (%d). Sending REJ(%d).\n", received_Ns, g_Nr);
            unsigned char rej_c = (g_Nr == 0) ? C_REJ_0 : C_REJ_1;
            unsigned char rej_frame[SU_FRAME_SIZE];
            create_su_frame(rej_frame, A_TX, rej_c);
            writeBytesSerialPort(rej_frame, SU_FRAME_SIZE);
            return 0; // RETURN 0: Frame discarded, application should retry llread.
        }
    }
    
    // 3. De-stuffing: Process only the stuffed data/BCC2 section
    int stuffed_data_size = frame_idx - 5; // Frame size - (F, A, C, BCC1, F)
    int destuffed_size = byte_destuffing(&frame_buf[data_start_idx], stuffed_data_size, destuffed_payload_and_bcc2);

    if (destuffed_size < 1) { // Error or just BCC2 received
        fprintf(stderr, "Rx: De-stuffing failed or payload too short.\n");
        // Frame received but corrupted internally. Send REJ.
        unsigned char rej_c = (g_Nr == 0) ? C_REJ_0 : C_REJ_1;
        unsigned char rej_frame[SU_FRAME_SIZE];
        create_su_frame(rej_frame, A_TX, rej_c);
        writeBytesSerialPort(rej_frame, SU_FRAME_SIZE);
        return 0; // RETURN 0: Frame discarded, application should retry llread.
    }
    
    // The last byte is the received BCC2
    int data_size = destuffed_size - 1; 
    unsigned char received_bcc2 = destuffed_payload_and_bcc2[data_size];

    // 4. Calculate BCC2 on the received data payload
    unsigned char calculated_bcc2 = calculate_bcc2(destuffed_payload_and_bcc2, data_size);

    // 5. Check BCC2
    if (calculated_bcc2 != received_bcc2) {
        // BCC2 Error: Send REJ for the current sequence number (g_Nr)
        fprintf(stderr, "Rx: BCC2 Checksum Error (Got 0x%02x, Expected 0x%02x). Sending REJ(%d).\n", 
                received_bcc2, calculated_bcc2, g_Nr);
        unsigned char rej_c = (g_Nr == 0) ? C_REJ_0 : C_REJ_1;
        unsigned char rej_frame[SU_FRAME_SIZE];
        create_su_frame(rej_frame, A_TX, rej_c);
        writeBytesSerialPort(rej_frame, SU_FRAME_SIZE);
        return 0; // RETURN 0: Frame discarded, application should retry llread.
    }

    // --- SUCCESS: Valid frame received, acknowledged, and data ready ---

    printf("Rx: Frame valid! BCC1/BCC2 OK. Data size: %d. Sending RR(%d) acknowledgement.\n", data_size, 1 - g_Nr);

    // 6. Send RR(Nr+1) Acknowledgment
    unsigned char next_Nr = 1 - g_Nr;
    unsigned char rr_c = (next_Nr == 0) ? C_RR_0 : C_RR_1; // RR0 if we expect I(0) next, RR1 if we expect I(1) next
    unsigned char rr_frame[SU_FRAME_SIZE];
    create_su_frame(rr_frame, A_TX, rr_c);
    writeBytesSerialPort(rr_frame, SU_FRAME_SIZE);

    // 7. Update next expected sequence number (Nr)
    g_Nr = next_Nr;

    // 8. Copy payload to the output packet buffer
    memcpy(packet, destuffed_payload_and_bcc2, data_size);
    
    return data_size; // Return the number of data bytes delivered
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose()
{
    if (fd1 == -1) return -1;

    int res = -1;

    if (g_role == TX) {
        printf("Tx: Initiating disconnection...\n");
        unsigned char disc_frame_tx[SU_FRAME_SIZE];
        
        // 1. TX sends DISC (Command, uses A_TX address)
        create_su_frame(disc_frame_tx, A_TX, C_DISC);
        
        for (int i = 1; i <= MAX_ATTEMPTS; i++) {
            g_alarm_flag = 0;
            printf("Tx: Sending DISC frame (attempt %d/%d)...\n", i, MAX_ATTEMPTS);
            // CORRECTED: Use return value of writeBytesSerialPort
            if (writeBytesSerialPort(disc_frame_tx, SU_FRAME_SIZE) < 0) { 
                fprintf(stderr, "Tx: Error writing DISC frame.\n");
            }

            // 2. Wait for Receiver's DISC reply (Command from RX, uses A_RX address)
            if (llwait_for_frame(A_RX, C_DISC)) { 
                printf("Tx: Received DISC frame from Receiver. Sending final UA.\n");
                
                // 3. Send final UA frame (Reply to RX's command, uses A_TX address)
                unsigned char ua_frame_tx[SU_FRAME_SIZE];
                create_su_frame(ua_frame_tx, A_TX, C_UA); 
                // CORRECTED: Use return value of writeBytesSerialPort
                if (writeBytesSerialPort(ua_frame_tx, SU_FRAME_SIZE) < 0) {
                     fprintf(stderr, "Tx: Error writing final UA frame.\n");
                }

                printf("Tx: Connection closed gracefully.\n");
                res = 0;
                break; // Handshake complete
            } else {
                if (i < MAX_ATTEMPTS) {
                    printf("Tx: Timeout. Will retry.\n");
                } else {
                    fprintf(stderr, "Tx: Failed to complete DISC handshake after %d attempts.\n", MAX_ATTEMPTS);
                    res = -1;
                }
            }
        }
    } 
    else if (g_role == RX) {
        printf("Rx: Waiting for DISC frame...\n");
        
        // 1. Wait for Transmitter's initial DISC (Command from TX, uses A_TX address)
        if (llwait_for_frame(A_TX, C_DISC)) {
            printf("Rx: Received valid DISC frame. Sending own DISC and waiting for final UA.\n");
            
            // 2. Send own DISC frame as reply (Command from RX, uses A_RX address)
            unsigned char disc_frame_rx[SU_FRAME_SIZE];
            create_su_frame(disc_frame_rx, A_RX, C_DISC); 
            
            // Send DISC multiple times until UA is received, or fail
            for (int i = 1; i <= MAX_ATTEMPTS; i++) {
                g_alarm_flag = 0;
                // CORRECTED: Use return value of writeBytesSerialPort
                if (writeBytesSerialPort(disc_frame_rx, SU_FRAME_SIZE) < 0) {
                    fprintf(stderr, "Rx: Error writing DISC frame.\n");
                }
                printf("Rx: Sent DISC frame (attempt %d/%d). Waiting for final UA...\n", i, MAX_ATTEMPTS);
                
                // 3. Wait for Transmitter's final UA (Reply from TX, uses A_TX address)
                if (llwait_for_frame(A_TX, C_UA)) {
                    printf("Rx: Received final UA. Connection closed gracefully.\n");
                    res = 0;
                    break;
                } else if (i == MAX_ATTEMPTS) {
                    printf("Rx: Timed out waiting for final UA after %d attempts, closing connection anyway.\n", MAX_ATTEMPTS);
                    res = -1;
                }
            }
        } else {
            // Timed out waiting for the initial DISC from TX
            fprintf(stderr, "Rx: Timed out waiting for DISC frame. Aborting connection.\n");
            res = -1;
        }
    }

    // Common cleanup
    if (fd1 != -1) {
        disableAlarm(); 
        clearAlarm();
        close(fd1); 
        fd1 = -1;
        printf("Serial port closed.\n");
    }

    return res;
}