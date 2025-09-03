// combined_dxl.h

#pragma once
#include <stdint.h>

void setupDXL();
void moveGripper(int32_t goal_velocity, float revolutions, int direction);   
void moveDispenser(int32_t goal_velocity, float revolutions, int direction);
void Station(int station_num, int32_t goal_velocity, float revolutions, int direction);
void closeDXL();
