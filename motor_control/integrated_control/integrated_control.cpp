#include "combined_arduinos.h"
#include "motor_controller.h"
#include "combined_dxl.h"

#include <windows.h>
#include <iostream>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <inttypes.h>

const float THETA2_OFFSET_DEG = 25.0f;	// initial angle for joint 2

int main() {
    initArduinos();                     // set arduinos
    setupDXL();                         // set gripper, dispenser dxl
    printf("\nSet initial settings\n");

    sendToLinear(2, 0, 0);             // Linear motor initialization
    setManipulatorControl();            // robot arm initialization
    cubic_trajectory_fk_two_links(-38, (0-THETA2_OFFSET_DEG), 1.0f);
    Sleep(1000);

    printf("--------------- Initialization complete! ---------------\n");

    int select_mode;

    int dir, distance_mm, volume;
    int32_t goal_vel;
    float rev;
    float deg[2] = { 0 };
    float vel[2] = { 0 };
    float tq[2] = { 0 };
    float eetarget[2];
    int func;
    int test;
    int repeat;

    while (1) {
        printf("\n== Select the mode ( 1: Arm | 2: Linear | 3: Gripper | 4: Dispenser | 5: Icemaker | 6: CoffeeMachine | 7:SYRUP | 8: STATION | 9: LED | 10:EXIT ) ==\n");
        scanf_s("%d", &select_mode);

        /// ARM CONTROL ///
        if (select_mode == 1) {
            int arm_con_mode;
            printf("== Select the control mode (1: Input degree & FK | 2: Input x, y goal & IK) ==\n");
            scanf_s("%d", &arm_con_mode);

            printf("\nCurrent link velocity, torque value (vel1, tq1, vel2, tq2): %.3f, %.3f, %.3f, %.3f\n", vel[0], tq[0], vel[1], tq[1]);
            printf("Do you want to continue with these values? [y/n]: ");

            char response = getchar(); // get leftover '\n'
            response = getchar();

            if (response == 'n' || response == 'N') {
                printf("Enter vel1 tq1 vel2 tq2: ");
                scanf_s("%f %f %f %f", &vel[0], &tq[0], &vel[1], &tq[1]);
                printf("Values updated!\n");
            }

            if (arm_con_mode == 1) { // FK
                printf("Enter deg1 deg2: ");
                scanf_s("%f %f", &deg[0], &deg[1]);

                cubic_trajectory_fk_one_link(1, deg[0], 1.5);
                cubic_trajectory_fk_one_link(2, deg[1], 1);

                forward_kinematics(deg[0], deg[1]);
                Sleep(1000);
            }
            else if (arm_con_mode == 2) { // IK
                printf("Enter target x y: ");
                scanf_s("%f %f", &eetarget[0], &eetarget[1]);

                float ik_theta1_deg, ik_theta2_deg;

                if (inverse_kinematics(eetarget[0], eetarget[1], &ik_theta1_deg, &ik_theta2_deg)) {
                    cubic_trajectory_fk_one_link(1, ik_theta1_deg, 1);
                    cubic_trajectory_fk_one_link(2, ik_theta2_deg, 1.5);
                    
                    Sleep(500);
                }
                else {
                    printf("IK failed: target out of range.\n");
                }
                Sleep(1000);
            }

        }
        /// LINEAR MOTOR ///
        else if (select_mode == 2) {
            printf("Enter direction (0: down, 1: up) and distance (mm): ");
            scanf_s("%d %d", &dir, &distance_mm);
            sendToLinear(2, dir, distance_mm);
            Sleep(1000);
        }
        /// GRIPPER ///
        else if (select_mode == 3) {
            printf("Enter gripper (float)velocity: ");
            scanf_s("%" SCNd32, &goal_vel);
            printf("Enter gripper (float)revolutions (int)direction [(-1): CCW = open | (+1): CW = close]: ");
            scanf_s("%f %d", &rev, &dir);
            moveGripper(goal_vel, rev, dir);

            Sleep(1000);
        }
        /// DISPENSER ///
        else if (select_mode == 4) {
            printf("Enter dispenser (float)velocity: ");
            scanf_s("%" SCNd32, &goal_vel);
            printf("Enter dispenser (float)revolutions (int)direction [(-1): CCW = open | (+1): CW = close]: ");
            scanf_s("%f %d", &rev, &dir);
            moveDispenser(goal_vel, rev, dir);

            Sleep(1000);
        }
        /// ICEMAKER ///
        else if (select_mode == 5) {
            printf("Enter Ice Maker function num (1: hot | 2: cold | 3: water | 4: ice): \n");
            scanf_s("%d %d", &func, &repeat);
            sendToicemaker(func, repeat);

            Sleep(3000);
        }
        /// COFFEEMACHINE ///
        else if (select_mode == 6) {
            printf("Enter Coffee Machine function num (1: hot water | 2: espresso | 3: soft americano | 4: americano): \n");
            scanf_s("%d", &func);
            sendTocoffeemachine(func);

            Sleep(3000);
        }
        /// SYRUP ///
        else if (select_mode == 7) {
            printf("Enter direction (0: out, 1: in) and volume (mm): ");
            scanf_s("%d %d", &dir, &volume);
            sendToLinear(1, dir, volume);
            Sleep(1000);
        }
        /// STATION ///
        else if (select_mode == 8) {
            printf("Enter station (float)velocity: ");
            scanf_s("%" SCNd32, &goal_vel);
            printf("Enter station (float)revolutions (int)direction [(-1): CCW = open | (+1): CW = close]: ");
            scanf_s("%f %d", &rev, &dir);
            moveThirdStation(goal_vel, rev, dir);

            Sleep(1000);
        }
        /// LED  ///
        else if (select_mode == 9) {
            printf("Enter LED function num (9: FIRST | 10: SECOND | 11: THIRD | 12: FOURTH): \n");
            scanf_s("%d", &func);
            sendToLED(func);
            Sleep(1000);
        }
        /// EXIT ///
        else if (select_mode == 10) {
            break;
        }
        else {
            printf("Invalid input.\n");
        }
    }
    closeDispenser();
    closeGripper();
    closeSerials();
    CAN_close();

    return 0;
}

