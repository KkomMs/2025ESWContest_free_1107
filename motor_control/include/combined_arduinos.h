// control_interface.h
#pragma once

void initArduinos();
void sendToLinear(int motor, int direction, int distance_mm);
void sendToicemaker(int function_num, int repeat);
void sendTocoffeemachine(int function_num);
void sendTosyrup(int function_num);
void sendToLED(int function_num);
void closeSerials();
