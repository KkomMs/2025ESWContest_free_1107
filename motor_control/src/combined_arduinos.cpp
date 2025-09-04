#include "combined_arduinos.h"
#include <windows.h>
#include <stdio.h>
#include <string>

// COM4 : Linear motor + Ice Maker
// COM6 : Coffee Machine

static HANDLE hSerial_COM4;
static HANDLE hSerial_COM6;

HANDLE setupSerial(const char* port) {
    HANDLE hSerial = CreateFileA((std::string("\\\\.\\") + port).c_str(),
        GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Arduino connect failed (%s)\n", port);
        exit(1);
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    SetCommState(hSerial, &dcb);

    printf("Arduino connection success: %s\n", port);
    return hSerial;
}

void initArduinos() {
    hSerial_COM4 = setupSerial("COM4");
    hSerial_COM6 = setupSerial("COM6");
}

void sendToLinear(int motor, int direction, int distance_mm) {
    char cmd[32];
    sprintf_s(cmd, "%d %d %d\n", motor, direction, distance_mm);
    DWORD bytesWritten;
    WriteFile(hSerial_COM4, cmd, strlen(cmd), &bytesWritten, NULL);
    printf("Send to Linear: %s", cmd);
}

void sendToicemaker(int function_num, int repeat) {
    if (function_num == 4) {  // ice
        for (int i = 0; i <= repeat; i++) {
            char cmd[16];
            sprintf_s(cmd, "%d\n", function_num);
            DWORD bytesWritten;
            WriteFile(hSerial_COM4, cmd, strlen(cmd), &bytesWritten, NULL);
        }
    }
    else if (function_num == 3) {   // water
        for (int i = 0; i <= repeat; i++) {
            char cmd[16];
            sprintf_s(cmd, "%d\n", function_num);
            DWORD bytesWritten;
            WriteFile(hSerial_COM4, cmd, strlen(cmd), &bytesWritten, NULL);
        }
    }
    else {
        char cmd[16];
        sprintf_s(cmd, "%d\n", function_num);
        DWORD bytesWritten;
        WriteFile(hSerial_COM4, cmd, strlen(cmd), &bytesWritten, NULL);
    }
}

void sendTocoffeemachine(int function_num) {
    char cmd[16];
    sprintf_s(cmd, "%d\n", function_num);
    DWORD bytesWritten;
    WriteFile(hSerial_COM6, cmd, strlen(cmd), &bytesWritten, NULL);
}

void sendToLED(int function_num) {
    char cmd[16];
    sprintf_s(cmd, "%d\n", function_num);
    DWORD bytesWritten;
    WriteFile(hSerial_COM6, cmd, strlen(cmd), &bytesWritten, NULL);
}

void closeSerials() {
    CloseHandle(hSerial_COM4);
    CloseHandle(hSerial_COM6);
}
