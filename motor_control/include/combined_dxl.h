// combined_dxl.h

#pragma once
#include <stdint.h>

void setupDXL();
void moveGripper(int32_t goal_velocity, float revolutions, int direction);   
void moveDispenser(int32_t goal_velocity, float revolutions, int direction); 
void moveFirstStation(int32_t goal_velocity, float revolutions, int direction);
void moveSecondStation(int32_t goal_velocity, float revolutions, int direction);
void moveThirdStation(int32_t goal_velocity, float revolutions, int direction);
void moveFourthStation(int32_t goal_velocity, float revolutions, int direction);
void closeGripper();
void closeDispenser();
