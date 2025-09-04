#include "combined_dxl.h"
#include <conio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <math.h>
#include "dynamixel_sdk.h"

#define MX_SERIES       // gripper, dispenser MX-64R

// control table address
#define ADDR_OPERATING_MODE         11
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define ADDR_GOAL_VELOCITY          104         // -vel_limit ~ vel_limit
#define ADDR_PRESENT_VELOCITY       128
#define VELOCITY_LIMIT              415         // limit of goal velocity (0.229rpm, range: 0 ~ 1,023)

#define BAUDRATE                    57600
#define PROTOCOL_VERSION            2.0
#define DISPENSER_DXL_ID            6           // Dispenser ID: 0
#define GRIPPER_DXL_ID              7           // Gripper ID: 1
///  station  ///
#define FIRST_STATION_DXL_ID            8           // FIRST_STATION ID: 8
#define SECOND_STATION_DXL_ID           9           // SECOND_STATION ID: 9
#define THIRD_STATION_DXL_ID            10           // THIRD_STATION ID: 10
#define FOURTH_STATION_DXL_ID           11           // FOURTH_STATION ID: 11

#define VELOCITY_MODE               1           // velocity control mode (infinite rev.)
#define EXT_POSITION_MODE           4           // extended position control mode. -256 ~ 256 rev.
#define DEVICENAME                  "COM5"

#define TORQUE_ENABLE               1
#define TORQUE_DISABLE              0
#define DXL_MOVING_STATUS_THRESHOLD 20

#define TICKS_PER_REV     4096                  // 1 rev = 4096 ticks
#define RPM_TO_UNIT       0.229f                // [rev/min] per 1 unit of goal velocity
#define REV_EPSILON       0.005f                // 회전수 정지 오차 허용

dynamixel::PortHandler* portHandler;
dynamixel::PacketHandler* packetHandler;
int32_t grip_dxl_present_position = 0;
int32_t disp_dxl_present_position = 0;
///  station  ///
int32_t first_station_dxl_present_position = 0;
int32_t second_station_dxl_present_position = 0;
int32_t third_station_dxl_present_position = 0;
int32_t fourth_station_dxl_present_position = 0;

static inline float ticks_to_revs(int32_t ticks) {
    return (float)ticks / (float)TICKS_PER_REV;
}

void setupDXL() {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        printf("Failed to open port!\n");
        exit(1);
    }

    // 공용 Baudrate 설정
    if (!portHandler->setBaudRate(BAUDRATE)) {
        printf("Error :: Failed to set baudrate!\n");
        exit(1);
    }
    Sleep(100);

    /// initialize gripper ///
    {
        uint8_t grip_dxl_error = 0;

        // Torque OFF
        packetHandler->write1ByteTxRx(portHandler, GRIPPER_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &grip_dxl_error);
        // Velocity Mode
        packetHandler->write1ByteTxRx(portHandler, GRIPPER_DXL_ID, ADDR_OPERATING_MODE, VELOCITY_MODE, &grip_dxl_error);
        // Torque ON
        packetHandler->write1ByteTxRx(portHandler, GRIPPER_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &grip_dxl_error);

        printf("Gripper initialized successfully.\n");
    }

    /// initialize dispenser ///
    {
        uint8_t disp_dxl_error = 0;

        // Torque OFF
        packetHandler->write1ByteTxRx(portHandler, DISPENSER_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &disp_dxl_error);
        // Velocity Mode
        packetHandler->write1ByteTxRx(portHandler, DISPENSER_DXL_ID, ADDR_OPERATING_MODE, VELOCITY_MODE, &disp_dxl_error);
        // Torque ON
        packetHandler->write1ByteTxRx(portHandler, DISPENSER_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &disp_dxl_error);

        printf("Dispenser initialized successfully.\n");
    }
    ///  station  ///
    /// initialize first_station ///
    {
        uint8_t err = 0;
        packetHandler->write1ByteTxRx(portHandler, FIRST_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
        packetHandler->write1ByteTxRx(portHandler, FIRST_STATION_DXL_ID, ADDR_OPERATING_MODE, VELOCITY_MODE, &err);
        packetHandler->write1ByteTxRx(portHandler, FIRST_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &err);
        printf("first_station initialized successfully.\n");
    }
    /// initialize second_station ///
    {
        uint8_t err = 0;
        packetHandler->write1ByteTxRx(portHandler, SECOND_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
        packetHandler->write1ByteTxRx(portHandler, SECOND_STATION_DXL_ID, ADDR_OPERATING_MODE, VELOCITY_MODE, &err);
        packetHandler->write1ByteTxRx(portHandler, SECOND_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &err);
        printf("second_station initialized successfully.\n");
    }
    /// initialize third_station ///
    {
        uint8_t err = 0;
        packetHandler->write1ByteTxRx(portHandler, THIRD_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
        packetHandler->write1ByteTxRx(portHandler, THIRD_STATION_DXL_ID, ADDR_OPERATING_MODE, VELOCITY_MODE, &err);
        packetHandler->write1ByteTxRx(portHandler, THIRD_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &err);
        printf("third_station initialized successfully.\n");
    }
    /// initialize fourth_station ///
    {
        uint8_t err = 0;
        packetHandler->write1ByteTxRx(portHandler, FOURTH_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
        packetHandler->write1ByteTxRx(portHandler, FOURTH_STATION_DXL_ID, ADDR_OPERATING_MODE, VELOCITY_MODE, &err);
        packetHandler->write1ByteTxRx(portHandler, FOURTH_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &err);
        printf("fourth_station initialized successfully.\n");
    }

}

// ===================== GRIPPER =====================
void moveGripper(int32_t goal_velocity, float revolutions, int direction) {
    if (goal_velocity > VELOCITY_LIMIT || goal_velocity < -VELOCITY_LIMIT) {
        printf("Gripper :: Invalid velocity. [Velocity limit: %d]\n", VELOCITY_LIMIT);
        return;
    }
    if (direction != -1 && direction != +1) {
        printf("Gripper :: Invalid direction. Use -1 (CW) or +1 (CCW).\n");
        return;
    }
    if (revolutions <= 0.0f) {
        printf("Gripper :: revolutions must be > 0.\n");
        return;
    }

    // 현재 위치 읽기
    uint8_t err = 0;
    int res = packetHandler->read4ByteTxRx(
        portHandler, GRIPPER_DXL_ID, ADDR_PRESENT_POSITION,
        (uint32_t*)&grip_dxl_present_position, &err
    );
    if (res != COMM_SUCCESS) {
        printf("Gripper :: Failed to read position.\n");
        return;
    }
    printf("Gripper :: Start position: %ld\n", (long)grip_dxl_present_position);

    // 목표 tick 계산 (기존 부호 체계 유지: 현재 - dir * delta)
    const int32_t goal_ticks_delta = direction * (int32_t)(revolutions * TICKS_PER_REV);
    const int32_t goal_pos = grip_dxl_present_position - goal_ticks_delta;
    const float   goal_rpm = (float)goal_velocity * RPM_TO_UNIT;

    printf("Gripper :: v=%ld (%.3f rpm), target=%.3f rev, dir=%s, goal_ticks=%ld\n",
        (long)goal_velocity, goal_rpm, revolutions,
        (direction == -1 ? "CW" : "CCW"),                                  // CHANGED: -1=CW, +1=CCW
        (long)goal_pos);

    // 속도 명령
    res = packetHandler->write4ByteTxRx(portHandler, GRIPPER_DXL_ID, ADDR_GOAL_VELOCITY,
        goal_velocity, &err);
    if (res != COMM_SUCCESS) {
        printf("Gripper :: Failed to write goal velocity.\n");
        return;
    }

    // 진행 모니터링 (회전수 기준 소프트 정지)
    int32_t curr_pos = grip_dxl_present_position;
    do {
        res = packetHandler->read4ByteTxRx(portHandler, GRIPPER_DXL_ID, ADDR_PRESENT_POSITION,
            (uint32_t*)&curr_pos, &err);
        if (res != COMM_SUCCESS) break;

        int32_t moved_ticks = curr_pos - grip_dxl_present_position;
        float moved_revs = fabsf(ticks_to_revs(moved_ticks));

        //printf("Gripper :: GoalTicks:%ld | Curr:%ld | Moved:%.3f rev\n",
            //(long)goal_pos, (long)curr_pos, moved_revs);

        if (moved_revs >= (revolutions - REV_EPSILON)) break;           // NEW
        Sleep(100);
    } while (abs(goal_pos - curr_pos) > DXL_MOVING_STATUS_THRESHOLD);

    // 정지
    res = packetHandler->write4ByteTxRx(portHandler, GRIPPER_DXL_ID, ADDR_GOAL_VELOCITY, 0, &err);
    if (res != COMM_SUCCESS) {
        printf("Gripper :: Failed to stop motor.\n");
        return;
    }
    printf("Gripper :: %.3f revolution completed and stopped.\n", revolutions);
}

// ===================== DISPENSER =====================
void moveDispenser(int32_t goal_velocity, float revolutions, int direction) {
    if (goal_velocity > VELOCITY_LIMIT || goal_velocity < -VELOCITY_LIMIT) {
        printf("Dispenser :: Invalid velocity. [Velocity limit: %d]\n", VELOCITY_LIMIT);
        return;
    }
    if (direction != -1 && direction != +1) {
        printf("Dispenser :: Invalid direction. Use -1 (CW) or +1 (CCW).\n");
        return;
    }
    if (revolutions <= 0.0f) {
        printf("Dispenser :: revolutions must be > 0.\n");
        return;
    }

    // 현재 위치 읽기
    uint8_t err = 0;
    int res = packetHandler->read4ByteTxRx(
        portHandler, DISPENSER_DXL_ID, ADDR_PRESENT_POSITION,
        (uint32_t*)&disp_dxl_present_position, &err
    );
    if (res != COMM_SUCCESS) {
        printf("Dispenser :: Failed to read position.\n");
        return;
    }
    printf("Dispenser :: Start position: %ld\n", (long)disp_dxl_present_position);

    // 목표 tick 계산 (기존 부호 체계 유지: 현재 - dir * delta)
    const int32_t goal_ticks_delta = direction * (int32_t)(revolutions * TICKS_PER_REV);
    const int32_t goal_pos = disp_dxl_present_position - goal_ticks_delta;
    const float   goal_rpm = (float)goal_velocity * RPM_TO_UNIT;

    printf("Dispenser :: v=%ld (%.3f rpm), target=%.3f rev, dir=%s, goal_ticks=%ld\n",
        (long)goal_velocity, goal_rpm, revolutions,
        (direction == -1 ? "CW" : "CCW"),                                  // CHANGED
        (long)goal_pos);

    // 속도 명령
    res = packetHandler->write4ByteTxRx(portHandler, DISPENSER_DXL_ID, ADDR_GOAL_VELOCITY,
        goal_velocity, &err);
    if (res != COMM_SUCCESS) {
        printf("Dispenser :: Failed to write goal velocity.\n");
        return;
    }

    // 진행 모니터링 (회전수 기준 소프트 정지)
    int32_t curr_pos = disp_dxl_present_position;
    do {
        res = packetHandler->read4ByteTxRx(portHandler, DISPENSER_DXL_ID, ADDR_PRESENT_POSITION,
            (uint32_t*)&curr_pos, &err);
        if (res != COMM_SUCCESS) break;

        int32_t moved_ticks = curr_pos - disp_dxl_present_position;
        float moved_revs = fabsf(ticks_to_revs(moved_ticks));

        //printf("Dispenser :: GoalTicks:%ld | Curr:%ld | Moved:%.3f rev\n",
            //(long)goal_pos, (long)curr_pos, moved_revs);

        if (moved_revs >= (revolutions - REV_EPSILON)) break;           // NEW
        Sleep(100);
    } while (abs(goal_pos - curr_pos) > DXL_MOVING_STATUS_THRESHOLD);

    // 정지
    res = packetHandler->write4ByteTxRx(portHandler, DISPENSER_DXL_ID, ADDR_GOAL_VELOCITY, 0, &err);
    if (res != COMM_SUCCESS) {
        printf("Dispenser :: Failed to stop motor.\n");
        return;
    }
    printf("Dispenser :: %.3f revolution completed and stopped.\n", revolutions);
}
///  station  ///
void moveFirstStation(int32_t goal_velocity, float revolutions, int direction) {
    if (goal_velocity > VELOCITY_LIMIT || goal_velocity < -VELOCITY_LIMIT) {
        printf("first_station :: Invalid velocity. [Velocity limit: %d]\n", VELOCITY_LIMIT);
        return;
    }
    if (direction != -1 && direction != +1) {
        printf("first_station :: Invalid direction. Use -1 (CW) or +1 (CCW).\n");
        return;
    }
    if (revolutions <= 0.0f) {
        printf("first_station :: revolutions must be > 0.\n");
        return;
    }

    uint8_t err = 0;
    int res = packetHandler->read4ByteTxRx(
        portHandler, FIRST_STATION_DXL_ID, ADDR_PRESENT_POSITION,
        (uint32_t*)&first_station_dxl_present_position, &err
    );
    if (res != COMM_SUCCESS) { printf("first_station :: Failed to read position.\n"); return; }
    printf("first_station :: Start position: %ld\n", (long)first_station_dxl_present_position);

    const int32_t goal_ticks_delta = direction * (int32_t)(revolutions * TICKS_PER_REV);
    const int32_t goal_pos = first_station_dxl_present_position - goal_ticks_delta;
    const float   goal_rpm = (float)goal_velocity * RPM_TO_UNIT;

    printf("first_station :: v=%ld (%.3f rpm), target=%.3f rev, dir=%s, goal_ticks=%ld\n",
        (long)goal_velocity, goal_rpm, revolutions, (direction == -1 ? "CW" : "CCW"), (long)goal_pos);

    res = packetHandler->write4ByteTxRx(portHandler, FIRST_STATION_DXL_ID, ADDR_GOAL_VELOCITY, goal_velocity, &err);
    if (res != COMM_SUCCESS) { printf("first_station :: Failed to write goal velocity.\n"); return; }

    int32_t curr_pos = first_station_dxl_present_position;
    do {
        res = packetHandler->read4ByteTxRx(portHandler, FIRST_STATION_DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&curr_pos, &err);
        if (res != COMM_SUCCESS) break;
        int32_t moved_ticks = curr_pos - first_station_dxl_present_position;
        float moved_revs = fabsf(ticks_to_revs(moved_ticks));
        if (moved_revs >= (revolutions - REV_EPSILON)) break;
        Sleep(100);
    } while (abs(goal_pos - curr_pos) > DXL_MOVING_STATUS_THRESHOLD);

    res = packetHandler->write4ByteTxRx(portHandler, FIRST_STATION_DXL_ID, ADDR_GOAL_VELOCITY, 0, &err);
    if (res != COMM_SUCCESS) { printf("first_station :: Failed to stop motor.\n"); return; }
    printf("first_station :: %.3f revolution completed and stopped.\n", revolutions);
}

void moveSecondStation(int32_t goal_velocity, float revolutions, int direction) {
    if (goal_velocity > VELOCITY_LIMIT || goal_velocity < -VELOCITY_LIMIT) {
        printf("second_station :: Invalid velocity. [Velocity limit: %d]\n", VELOCITY_LIMIT);
        return;
    }
    if (direction != -1 && direction != +1) {
        printf("second_station :: Invalid direction. Use -1 (CW) or +1 (CCW).\n");
        return;
    }
    if (revolutions <= 0.0f) {
        printf("second_station :: revolutions must be > 0.\n");
        return;
    }

    uint8_t err = 0;
    int res = packetHandler->read4ByteTxRx(
        portHandler, SECOND_STATION_DXL_ID, ADDR_PRESENT_POSITION,
        (uint32_t*)&second_station_dxl_present_position, &err
    );
    if (res != COMM_SUCCESS) { printf("second_station :: Failed to read position.\n"); return; }
    printf("second_station :: Start position: %ld\n", (long)second_station_dxl_present_position);

    const int32_t goal_ticks_delta = direction * (int32_t)(revolutions * TICKS_PER_REV);
    const int32_t goal_pos = second_station_dxl_present_position - goal_ticks_delta;
    const float   goal_rpm = (float)goal_velocity * RPM_TO_UNIT;

    printf("second_station :: v=%ld (%.3f rpm), target=%.3f rev, dir=%s, goal_ticks=%ld\n",
        (long)goal_velocity, goal_rpm, revolutions, (direction == -1 ? "CW" : "CCW"), (long)goal_pos);

    res = packetHandler->write4ByteTxRx(portHandler, SECOND_STATION_DXL_ID, ADDR_GOAL_VELOCITY, goal_velocity, &err);
    if (res != COMM_SUCCESS) { printf("second_station :: Failed to write goal velocity.\n"); return; }

    int32_t curr_pos = second_station_dxl_present_position;
    do {
        res = packetHandler->read4ByteTxRx(portHandler, SECOND_STATION_DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&curr_pos, &err);
        if (res != COMM_SUCCESS) break;
        int32_t moved_ticks = curr_pos - second_station_dxl_present_position;
        float moved_revs = fabsf(ticks_to_revs(moved_ticks));
        if (moved_revs >= (revolutions - REV_EPSILON)) break;
        Sleep(100);
    } while (abs(goal_pos - curr_pos) > DXL_MOVING_STATUS_THRESHOLD);

    res = packetHandler->write4ByteTxRx(portHandler, SECOND_STATION_DXL_ID, ADDR_GOAL_VELOCITY, 0, &err);
    if (res != COMM_SUCCESS) { printf("second_station :: Failed to stop motor.\n"); return; }
    printf("second_station :: %.3f revolution completed and stopped.\n", revolutions);
}

void moveThirdStation(int32_t goal_velocity, float revolutions, int direction) {
    if (goal_velocity > VELOCITY_LIMIT || goal_velocity < -VELOCITY_LIMIT) {
        printf("third_station :: Invalid velocity. [Velocity limit: %d]\n", VELOCITY_LIMIT);
        return;
    }
    if (direction != -1 && direction != +1) {
        printf("third_station :: Invalid direction. Use -1 (CW) or +1 (CCW).\n");
        return;
    }
    if (revolutions <= 0.0f) {
        printf("third_station :: revolutions must be > 0.\n");
        return;
    }

    uint8_t err = 0;
    int res = packetHandler->read4ByteTxRx(
        portHandler, THIRD_STATION_DXL_ID, ADDR_PRESENT_POSITION,
        (uint32_t*)&third_station_dxl_present_position, &err
    );
    if (res != COMM_SUCCESS) { printf("third_station :: Failed to read position.\n"); return; }
    printf("third_station :: Start position: %ld\n", (long)third_station_dxl_present_position);

    const int32_t goal_ticks_delta = direction * (int32_t)(revolutions * TICKS_PER_REV);
    const int32_t goal_pos = third_station_dxl_present_position - goal_ticks_delta;
    const float   goal_rpm = (float)goal_velocity * RPM_TO_UNIT;

    printf("third_station :: v=%ld (%.3f rpm), target=%.3f rev, dir=%s, goal_ticks=%ld\n",
        (long)goal_velocity, goal_rpm, revolutions, (direction == -1 ? "CW" : "CCW"), (long)goal_pos);

    res = packetHandler->write4ByteTxRx(portHandler, THIRD_STATION_DXL_ID, ADDR_GOAL_VELOCITY, goal_velocity, &err);
    if (res != COMM_SUCCESS) { printf("third_station :: Failed to write goal velocity.\n"); return; }

    int32_t curr_pos = third_station_dxl_present_position;
    do {
        res = packetHandler->read4ByteTxRx(portHandler, THIRD_STATION_DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&curr_pos, &err);
        if (res != COMM_SUCCESS) break;
        int32_t moved_ticks = curr_pos - third_station_dxl_present_position;
        float moved_revs = fabsf(ticks_to_revs(moved_ticks));
        if (moved_revs >= (revolutions - REV_EPSILON)) break;
        Sleep(100);
    } while (abs(goal_pos - curr_pos) > DXL_MOVING_STATUS_THRESHOLD);

    res = packetHandler->write4ByteTxRx(portHandler, THIRD_STATION_DXL_ID, ADDR_GOAL_VELOCITY, 0, &err);
    if (res != COMM_SUCCESS) { printf("third_station :: Failed to stop motor.\n"); return; }
    printf("third_station :: %.3f revolution completed and stopped.\n", revolutions);
}

void moveFourthStation(int32_t goal_velocity, float revolutions, int direction) {
    if (goal_velocity > VELOCITY_LIMIT || goal_velocity < -VELOCITY_LIMIT) {
        printf("fourth_station :: Invalid velocity. [Velocity limit: %d]\n", VELOCITY_LIMIT);
        return;
    }
    if (direction != -1 && direction != +1) {
        printf("fourth_station :: Invalid direction. Use -1 (CW) or +1 (CCW).\n");
        return;
    }
    if (revolutions <= 0.0f) {
        printf("fourth_station :: revolutions must be > 0.\n");
        return;
    }

    uint8_t err = 0;
    int res = packetHandler->read4ByteTxRx(
        portHandler, FOURTH_STATION_DXL_ID, ADDR_PRESENT_POSITION,
        (uint32_t*)&fourth_station_dxl_present_position, &err
    );
    if (res != COMM_SUCCESS) { printf("fourth_station :: Failed to read position.\n"); return; }
    printf("fourth_station :: Start position: %ld\n", (long)fourth_station_dxl_present_position);

    const int32_t goal_ticks_delta = direction * (int32_t)(revolutions * TICKS_PER_REV);
    const int32_t goal_pos = fourth_station_dxl_present_position - goal_ticks_delta;
    const float   goal_rpm = (float)goal_velocity * RPM_TO_UNIT;

    printf("fourth_station :: v=%ld (%.3f rpm), target=%.3f rev, dir=%s, goal_ticks=%ld\n",
        (long)goal_velocity, goal_rpm, revolutions, (direction == -1 ? "CW" : "CCW"), (long)goal_pos);

    res = packetHandler->write4ByteTxRx(portHandler, FOURTH_STATION_DXL_ID, ADDR_GOAL_VELOCITY, goal_velocity, &err);
    if (res != COMM_SUCCESS) { printf("fourth_station :: Failed to write goal velocity.\n"); return; }

    int32_t curr_pos = fourth_station_dxl_present_position;
    do {
        res = packetHandler->read4ByteTxRx(portHandler, FOURTH_STATION_DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&curr_pos, &err);
        if (res != COMM_SUCCESS) break;
        int32_t moved_ticks = curr_pos - fourth_station_dxl_present_position;
        float moved_revs = fabsf(ticks_to_revs(moved_ticks));
        if (moved_revs >= (revolutions - REV_EPSILON)) break;
        Sleep(100);
    } while (abs(goal_pos - curr_pos) > DXL_MOVING_STATUS_THRESHOLD);

    res = packetHandler->write4ByteTxRx(portHandler, FOURTH_STATION_DXL_ID, ADDR_GOAL_VELOCITY, 0, &err);
    if (res != COMM_SUCCESS) { printf("fourth_station :: Failed to stop motor.\n"); return; }
    printf("fourth_station :: %.3f revolution completed and stopped.\n", revolutions);
}


void closeGripper() {
    portHandler->setBaudRate(BAUDRATE);
    uint8_t grip_dxl_error = 0;
    packetHandler->write1ByteTxRx(portHandler, GRIPPER_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &grip_dxl_error);
    portHandler->closePort();
    printf("Gripper connection closed.\n");
}

void closeDispenser() {
    portHandler->setBaudRate(BAUDRATE);
    uint8_t disp_dxl_error = 0;
    packetHandler->write1ByteTxRx(portHandler, DISPENSER_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &disp_dxl_error);
    portHandler->closePort();
    printf("Dispenser connection closed.\n");
}
///  station  ///
void closeFirstStation() {
    portHandler->setBaudRate(BAUDRATE);
    uint8_t err = 0;
    packetHandler->write1ByteTxRx(portHandler, FIRST_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
    portHandler->closePort();
    printf("first_station connection closed.\n");
}

void closeSecondStation() {
    portHandler->setBaudRate(BAUDRATE);
    uint8_t err = 0;
    packetHandler->write1ByteTxRx(portHandler, SECOND_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
    portHandler->closePort();
    printf("second_station connection closed.\n");
}

void closeThirdStation() {
    portHandler->setBaudRate(BAUDRATE);
    uint8_t err = 0;
    packetHandler->write1ByteTxRx(portHandler, THIRD_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
    portHandler->closePort();
    printf("third_station connection closed.\n");
}

void closeFourthStation() {
    portHandler->setBaudRate(BAUDRATE);
    uint8_t err = 0;
    packetHandler->write1ByteTxRx(portHandler, FOURTH_STATION_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &err);
    portHandler->closePort();
    printf("fourth_station connection closed.\n");
}

