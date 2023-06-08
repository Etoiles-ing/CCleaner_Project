#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "i2c.h"

void I2C_Master_Transmit(uint8_t addr, uint8_t *buffer, uint8_t size) {
    // There must be at least 1 data byte (the register offset) and no more than 9 (to avoid overflowing the cmd buffer)
    if ((size < 1) || (size > 9)) {
        return;
    }

    // First part of the command up to the address (y flag is set to skip confirmations)
    char cmd[100];
    char tmp[10];
    sprintf(cmd, "i2ctransfer -y 1 w%d@0x%02x", size, addr >> 1);

    // Append register offset and data bytes to the command
    for (int i = 0; i < size; ++i) {
        sprintf(tmp, " 0x%02x", buffer[i]);
        strcat(cmd, tmp);
    }

    // Run command
    int ret = system(cmd);
    (void)ret;
}

void I2C_Master_Receive(uint8_t addr, uint8_t *buffer, uint8_t size) {
    // There must be at least 1 data byte read and no more than 8 (to avoid overflowing the cmd buffer)
    if ((size < 1) || (size > 8)) {
        return;
    }

    // Construct read command (y flag is set to skip confirmations)
    char cmd[100];
    sprintf(cmd, "i2ctransfer -y 1 r%d@0x%02x", size, addr >> 1);

    // Launch command
    FILE *fp;
    fp = popen(cmd, "r");
    if (fp == NULL) {
        printf("Failed to run command\n" );
        return;
    }

    // Read the output line
    char output[50];
    fgets(output, sizeof(output), fp);
    
    // Wait for child process to terminate
    pclose(fp);

    // Parse output to buffer
    for (int i = 0; i < size; ++i) {
        char* token = strtok((i == 0) ? output : NULL, " ");
        sscanf(token+2, "%2hhx", &buffer[i]);
    }
}
