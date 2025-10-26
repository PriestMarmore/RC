#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// Max size of the data field (payload) inside a data packet (Control, Sequence, L1, L2 + Data)
// We assume this is a standard chunk size for the data reading buffer.
#define MAX_DATA_PAYLOAD_SIZE 1024

// Packet Control Field Definitions
#define C_DATA 0x01
#define C_START 0x02
#define C_END 0x03
#define T_FILENAME 0x00
#define T_FILESIZE 0x01

// Helper function: Builds a START or END control packet
// type: C_START or C_END
// packet: buffer to store the resulting control packet
// filename: the name of the file
// fileSize: the size of the file
int build_control_packet(unsigned char *packet, unsigned char type, const char *filename, long fileSize) {
    if (!packet || !filename) return -1;

    int current_index = 0;

    // 1. Control Field (C)
    packet[current_index++] = type;

    // --- File Size TLV (Type-Length-Value) ---

    // T1: File Size Type
    packet[current_index++] = T_FILESIZE;

    // V1: File Size (8 bytes, standard long size)
    unsigned char fileSize_bytes[8];
    // Copy the long long size into a byte array (assuming little-endian architecture for simplicity)
    for (int i = 0; i < 8; i++) {
        fileSize_bytes[i] = (unsigned char)((fileSize >> (i * 8)) & 0xFF);
    }
    
    // L1: File Size Length (always 8 bytes for long)
    packet[current_index++] = 8;
    
    // V1: File Size Value
    memcpy(&packet[current_index], fileSize_bytes, 8);
    current_index += 8;

    // --- Filename TLV ---
    
    // T2: Filename Type
    packet[current_index++] = T_FILENAME;

    // L2: Filename Length
    size_t filename_len = strlen(filename);
    packet[current_index++] = (unsigned char)filename_len; // Filename length L2 (max 255)

    // V2: Filename Value
    memcpy(&packet[current_index], filename, filename_len);
    current_index += filename_len;

    return current_index;
}

// Helper function: Builds a DATA packet
// packet: buffer to store the resulting data packet
// data: buffer containing the file data chunk
// dataSize: size of the data chunk
// sequenceNumber: the current N field (0-255)
int build_data_packet(unsigned char *packet, const unsigned char *data, int dataSize, unsigned char sequenceNumber) {
    if (!packet || !data) return -1;

    int current_index = 0;

    // 1. Control Field (C)
    packet[current_index++] = C_DATA;

    // 2. Sequence Number (N)
    packet[current_index++] = sequenceNumber;

    // 3. Length Fields (L2, L1) - Data size is L2*256 + L1
    // We use a maximum of 1024 bytes, so L2 will typically be 4 or 0.
    packet[current_index++] = (unsigned char)(dataSize / 256); // L2 (MSB)
    packet[current_index++] = (unsigned char)(dataSize % 256); // L1 (LSB)

    // 4. Data (P)
    memcpy(&packet[current_index], data, dataSize);
    current_index += dataSize;

    return current_index;
}

// Forward declarations for internal use (or defined in .h)
int llwrite_application(const char *filename);
int llread_application(const char *filename_rx);


// -----------------------------------------------------------------------------
// APPLICATION LAYER ENTRY POINT (FIX FOR LINKER ERROR)
// -----------------------------------------------------------------------------

int applicationLayer(const char *serialPort, int baudRate, int role, const char *filename) {
    
    LinkLayer connectionParameters;
    strncpy(connectionParameters.serialPort, serialPort, sizeof(connectionParameters.serialPort) - 1);
    connectionParameters.baudRate = baudRate;
    connectionParameters.role = role;
    connectionParameters.nRetransmissions = 3; // Default value, adjust if needed
    connectionParameters.timeout = 4; // Default value, adjust if needed

    int fd = llopen(connectionParameters);
    if (fd < 0) {
        printf("Application: Error establishing link layer connection.\n");
        return -1;
    }

    int result = -1;
    if (role == TRANSMITTER) {
        result = llwrite_application(filename);
    } else if (role == RECEIVER) {
        result = llread_application(filename); // Pass default filename for Rx
    }

    if (llclose(fd) < 0) {
        printf("Application: Error closing link layer connection.\n");
        return -1;
    }
    
    return result;
}


// -----------------------------------------------------------------------------
// TRANSMITTER LOGIC
// -----------------------------------------------------------------------------

int llwrite_application(const char *filename) {
    // 1. Get File Information
    struct stat file_stat;
    if (stat(filename, &file_stat) == -1) {
        perror("Error getting file stats");
        return -1;
    }
    long file_size = file_stat.st_size;

    FILE *file_fd = fopen(filename, "rb");
    if (file_fd == NULL) {
        perror("Error opening file for reading");
        return -1;
    }

    // --- 1. SEND START CONTROL PACKET ---
    unsigned char start_packet[2048]; // Buffer large enough for control packet
    int start_packet_size = build_control_packet(start_packet, C_START, filename, file_size);
    
    printf("AppTx: Sending START packet (size: %d bytes, file: %s, size: %lu).\n", 
           start_packet_size, filename, (long unsigned)file_size);

    // FIX: Removed fd argument
    if (llwrite(start_packet, start_packet_size) < 0) {
        printf("AppTx: Error writing START packet. Aborting.\n");
        fclose(file_fd);
        return -1;
    }

    // --- 2. SEND DATA PACKETS (THE MISSING LOOP) ---
    unsigned char data_buffer[MAX_DATA_PAYLOAD_SIZE];
    int bytes_read;
    unsigned char sequence_number = 0; // Data packet sequence number (N)
    int total_data_sent = 0;
    
    // Loop to read file in chunks (MAX_DATA_PAYLOAD_SIZE), create data packets, and send them
    while ((bytes_read = fread(data_buffer, 1, MAX_DATA_PAYLOAD_SIZE, file_fd)) > 0) {
        
        // 2a. Build Data Packet
        unsigned char data_packet[MAX_DATA_PAYLOAD_SIZE + 4]; // 4 bytes for C, N, L2, L1
        int data_packet_size = build_data_packet(data_packet, data_buffer, bytes_read, sequence_number);

        // 2b. Send Data Packet using llwrite
        // FIX: Removed fd argument
        int llwrite_result = llwrite(data_packet, data_packet_size);

        if (llwrite_result < 0) {
            printf("AppTx: Error writing data packet. Aborting.\n");
            fclose(file_fd);
            return -1;
        }

        total_data_sent += bytes_read;
        sequence_number = (sequence_number + 1); // Sequence number implicitly wraps 0-255
        
        printf("AppTx: Sent data packet N=%d, bytes=%d. Total sent: %d/%lu\n", 
               (sequence_number - 1) & 0xFF, bytes_read, total_data_sent, (long unsigned)file_size);
    }
    
    // Check for file read errors
    if (ferror(file_fd)) {
        printf("AppTx: Error reading from file.\n");
        fclose(file_fd);
        return -1;
    }

    fclose(file_fd);

    // --- 3. SEND END CONTROL PACKET ---
    unsigned char end_packet[2048];
    int end_packet_size = build_control_packet(end_packet, C_END, filename, file_size);
    
    printf("AppTx: Sending END packet (size: %d bytes).\n", end_packet_size);

    // FIX: Removed fd argument
    if (llwrite(end_packet, end_packet_size) < 0) {
        printf("AppTx: Error writing END packet. Aborting.\n");
        return -1;
    }

    printf("AppTx: File transfer complete. Total bytes sent: %d.\n", total_data_sent);
    return 0;
}


// -----------------------------------------------------------------------------
// RECEIVER LOGIC
// -----------------------------------------------------------------------------

// Helper function: Parses a control packet (START or END)
long parse_control_packet(const unsigned char *packet, int size, char *filename_out, int max_filename_len) {
    if (size < 1) return -1;
    
    // Check Control Field (C) - not strictly necessary here but good practice
    unsigned char type = packet[0];
    if (type != C_START && type != C_END) return -1; 
    
    int current_index = 1;
    long file_size = -1;

    while (current_index < size) {
        unsigned char t_field = packet[current_index++];
        unsigned char l_field = packet[current_index++];
        
        if (current_index + l_field > size) {
            fprintf(stderr, "AppRx: Malformed TLV structure in control packet.\n");
            return -1;
        }

        if (t_field == T_FILESIZE && l_field == 8) {
            // Read 8 bytes for file size (V)
            file_size = 0;
            for (int i = 0; i < 8; i++) {
                file_size |= ((long)packet[current_index + i] << (i * 8));
            }
        } else if (t_field == T_FILENAME && l_field > 0 && filename_out) {
            // Read filename (V)
            int len = (l_field < max_filename_len - 1) ? l_field : max_filename_len - 1;
            memcpy(filename_out, &packet[current_index], len);
            filename_out[len] = '\0';
        }

        current_index += l_field;
    }
    
    return file_size;
}

// Helper function: Parses a data packet
// Returns the size of the data payload (P), or -1 on error.
int parse_data_packet(const unsigned char *packet, int size, unsigned char *data_out, unsigned char *sequence_number_out) {
    if (size < 4) return -1;

    // 1. Check Control Field (C)
    if (packet[0] != C_DATA) return -1;

    // 2. Sequence Number (N)
    *sequence_number_out = packet[1];

    // 3. Length Fields (L2, L1)
    unsigned int data_size = (packet[2] * 256) + packet[3]; // L2 * 256 + L1

    // 4. Check actual data length vs expected length
    if (size != 4 + data_size) {
        fprintf(stderr, "AppRx: Data packet length mismatch! Header says %u, actual is %d.\n", data_size, size - 4);
        return -1;
    }
    
    // 5. Copy Data (P)
    if (data_out) {
        memcpy(data_out, &packet[4], data_size);
    }

    return data_size;
}


int llread_application(const char *filename_rx) {
    // FIX: Using a statically allocated buffer for llread since it expects `unsigned char *`
    unsigned char packet_buffer[MAX_PACKET_SIZE]; // Assuming MAX_PACKET_SIZE is defined in link_layer.h or is large enough
    int packet_size;
    long expected_file_size = 0;
    long total_bytes_received = 0;
    char rx_filename[256];
    strncpy(rx_filename, filename_rx, sizeof(rx_filename) - 1); // Use the filename passed from main
    rx_filename[sizeof(rx_filename) - 1] = '\0';

    FILE *rx_file_fd = NULL;
    unsigned char expected_sequence_number = 0; // Next expected N

    // --- 1. RECEIVE START CONTROL PACKET ---
    // FIX: Passing the buffer directly, and llread will fill it and return size.
    packet_size = llread(packet_buffer);
    
    if (packet_size < 0 || packet_buffer[0] != C_START) {
        printf("AppRx: Failed to receive valid START packet. Aborting.\n");
        return -1;
    }

    expected_file_size = parse_control_packet(packet_buffer, packet_size, rx_filename, sizeof(rx_filename));
    
    if (expected_file_size < 0) {
        printf("AppRx: Failed to parse START packet.\n");
        return -1;
    }

    printf("AppRx: Received START packet. File: %s (Size: %lu bytes).\n", rx_filename, expected_file_size);

    // Open output file
    rx_file_fd = fopen(rx_filename, "wb");
    if (rx_file_fd == NULL) {
        perror("Error opening output file for writing");
        return -1;
    }

    // --- 2. RECEIVE DATA PACKETS ---
    while (total_bytes_received < expected_file_size) {
        // FIX: Passing the buffer directly, and llread will fill it and return size.
        packet_size = llread(packet_buffer);
        
        if (packet_size < 0) {
            printf("AppRx: Error reading data packet. Aborting.\n");
            fclose(rx_file_fd);
            return -1;
        }

        if (packet_buffer[0] == C_END) {
            // End packet received, break out of data loop
            break;
        }

        if (packet_buffer[0] == C_DATA) {
            unsigned char received_sequence_number;
            unsigned char data_buffer[MAX_DATA_PAYLOAD_SIZE];
            
            // Parse data packet
            int data_size = parse_data_packet(packet_buffer, packet_size, data_buffer, &received_sequence_number);
            
            if (data_size > 0) {
                if (received_sequence_number == expected_sequence_number) {
                    // Correct sequence number: write data and update expected sequence
                    size_t written = fwrite(data_buffer, 1, data_size, rx_file_fd);
                    if (written != data_size) {
                         perror("AppRx: Error writing data to file");
                         fclose(rx_file_fd);
                         return -1;
                    }
                    total_bytes_received += data_size;
                    expected_sequence_number = (expected_sequence_number + 1);
                    
                    printf("AppRx: Wrote %zu bytes from packet N=%d. Total received: %lu/%lu\n", 
                           written, received_sequence_number, total_bytes_received, expected_file_size);
                } else {
                    // Incorrect sequence number: discard data, link layer should handle NACK/REJ
                    printf("AppRx: Discarded packet N=%d (Expected N=%d).\n", 
                           received_sequence_number, expected_sequence_number);
                }
            }
        }
    }
    
    // --- 3. RECEIVE END CONTROL PACKET ---
    // If the loop broke due to C_END, we already have the packet in packet_buffer.
    
    if (packet_buffer[0] != C_END) {
        // We broke early (or reached file size exactly) and still need to read the END packet
        // FIX: Passing the buffer directly
        packet_size = llread(packet_buffer);
        if (packet_size < 0 || packet_buffer[0] != C_END) {
             printf("AppRx: Failed to receive valid END packet after data transfer.\n");
             fclose(rx_file_fd);
             return -1;
        }
    }
    
    parse_control_packet(packet_buffer, packet_size, NULL, 0); // Parse END packet (mostly for logging)
    printf("AppRx: Received END packet.\n");
    
    fclose(rx_file_fd);

    // --- 4. VERIFY AND CLOSE ---
    if (total_bytes_received != expected_file_size) {
        printf("AppRx: Transfer mismatch! Expected %lu bytes, received %lu bytes.\n", 
               expected_file_size, total_bytes_received);
        return -1;
    }

    printf("AppRx: File received and saved successfully: %s (%lu bytes).\n", rx_filename, total_bytes_received);
    return 0;
}
