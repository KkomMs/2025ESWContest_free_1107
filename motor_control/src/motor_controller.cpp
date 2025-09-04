#include "motor_controller.h"
#include "CAN_Access.h"
#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>

#define len 8
#define ext 0
#define rtr 0

// ===== ROBOT PARAMETER ===== //
const float L1 = 3.5f;	// 35cm * 0.1
const float L2 = 4.4f;	// 44cm * 0.1
const float THETA1_OFFSET_DEG = 14.0f;	// initial angle for joint 1
const float THETA2_OFFSET_DEG = 25.0f;	// initial angle for joint 2

CAN_HANDLE handle;
uint16_t CAN_id;

static inline float deg2rad(float deg) {	
	return deg * (float)M_PI / 180.0f;
}
static inline float rad2deg(float rad) {
	return rad * 180.0f / (float)M_PI;
}

int CAN_open(const char* name) {

	handle = CAN_OpenUsb(name);

	if (handle < 0) {
		printf("Failed to open USB2CAN\n");
		return 0;
	}
	printf("USB2CAN Opened!\n");

	return 1;
}

int CAN_close() {

	CAN_Close(handle);
	
	return 1;
}

void set_controller_mode(uint8_t nodeid, uint8_t mode, uint8_t param) {

	CAN_id = (nodeid << 5) + 0x0B; // CMD = 0x0B
	char data[8] = { 0 };

	data[0] = mode;
	data[4] = param;

	// send data
	if (CAN_Send(handle, CAN_id, len, data, ext, rtr)) {
		printf(">> Sent set controller mode : node = %d, control mode = %d, input mode = %d\n", nodeid, mode, param);
	}
	else {
		printf(">> Error :: set controller mode\n");
	}

}

void set_axis_state(uint8_t nodeid, uint8_t mode) {

	CAN_id = (nodeid << 5) + 0x07; // CMD = 0x07
	char data[8] = { 0 };

	data[0] = mode;

	// send data
	if (CAN_Send(handle, CAN_id, len, data, ext, rtr)) {
		printf(">> Sent set axis state : node = %d, state = %d\n", nodeid, mode);
	}
	else {
		printf(">> Error :: set axis state\n");
	}
}

void setManipulatorControl() {

	CAN_open("NTAGPJUW");
	CAN_SetTransferMode(handle, 1);
	CAN_SetTimeout(handle, 33, 100, 0);
	CAN_Purge(handle);

	set_controller_mode(1, 3, 1);
	set_controller_mode(2, 3, 5);
	set_axis_state(1, 8);
	set_axis_state(2, 8);
	Sleep(500);
}

void print_encoder_pos(uint8_t nodeid) {

	char data[8] = { 0 };

	// send data
	long recv_id;
	int recv_len;
	char recv_data[8];
	int recv_ext;
	int recv_rtr;
	int is_recv = 0;

	printf("+------------+---------------+\n");
	printf("|   CAN ID   |     Data      |\n");
	printf("+------------+---------------+\n");

	CAN_Purge(handle);		// remove queue

	Sleep(50);

	while (1) {
		if (CAN_CountRxQueue(handle) > 0) {
			if (CAN_Recv(handle, &recv_id, &recv_len, recv_data, &recv_ext, &recv_rtr)) {

				if (recv_id == (nodeid << 5) + 0x09) {
					printf("|     %02X     | ", recv_id);

					for (int i = 0; i < 4; i++) {
						printf(" %02X", (uint8_t)recv_data[i]);
					}
					printf("  |\n");
					printf("+------------+---------------+\n");

					is_recv++;

					if (is_recv == 1)
						break;
				}
			}
			else
				Sleep(1);
		}
	}
}

float encoder_pos_to_rev(uint8_t *data) {
	// convert 4 bytes to int32_t (little endian)
	uint32_t encoder_pos = ((int32_t)data[0]) | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 24);

	float rev = *(float*)&encoder_pos;

	return rev;
}

float get_motor_angle(int nodeid) {

	char data[8] = { 0 };

	// send data
	long recv_id;
	int recv_len;
	char recv_data[8];
	int recv_ext;
	int recv_rtr;
	int is_recv = 0;
	
	float rev;
	float curr_degree;

	CAN_Purge(handle);		// remove queue

	while (1) {
		if (CAN_CountRxQueue(handle) > 0) {
			if (CAN_Recv(handle, &recv_id, &recv_len, recv_data, &recv_ext, &recv_rtr)) {
				if (recv_id == (nodeid << 5) + 0x09) {
					rev = encoder_pos_to_rev((uint8_t*)recv_data);

					is_recv = 1;

					if (is_recv == 1)
						break;
				}
			}
			else
				Sleep(1);
		}
	}
	curr_degree = rev * 360.f / 8.0f;

	return curr_degree;
}

void print_current_motor_angle(int nodeid) {

	char data[8] = { 0 };

	// send data
	long recv_id;
	int recv_len;
	char recv_data[8];
	int recv_ext;
	int recv_rtr;
	int is_recv = 0;

	float rev;
	float curr_degree;

	CAN_Purge(handle);		// remove queue

	while (1) {
		if (CAN_CountRxQueue(handle) > 0) {
			if (CAN_Recv(handle, &recv_id, &recv_len, recv_data, &recv_ext, &recv_rtr)) {
				if (recv_id == (nodeid << 5) + 0x09) {
					rev = encoder_pos_to_rev((uint8_t*)recv_data);

					is_recv = 1;

					if (is_recv == 1)
						break;
				}
			}
			else
				Sleep(1);
		}
	}
	curr_degree = rev * 360.f / 8.0f;
	printf("Current %d motor rev, angle: %.3f rev, %.3f deg\n", nodeid, rev, curr_degree);
}

int set_input_pos(uint8_t nodeid, float pos_degree, float velff, float torqueff) {

	CAN_id = (nodeid << 5) + 0x0C; // CMD = 0x0C
	char data[8] = { 0 };

	if (pos_degree < -180.0f || pos_degree > 180.0f) {
		("\n");
		return 1;
	}

	float pos_rev = pos_degree * 8.0f / 360.0f;

	union { float f; uint32_t u; } tmp;
    tmp.f = pos_rev;
    uint32_t pos_rev32 = tmp.u;
	
	data[0] = (pos_rev32 >> 0) & 0xFF;
	data[1] = (pos_rev32 >> 8) & 0xFF;
	data[2] = (pos_rev32 >> 16) & 0xFF;
	data[3] = (pos_rev32 >> 24) & 0xFF;

	int16_t velff_scaled = (int16_t)(velff);
	data[4] = (velff_scaled >> 0) & 0xFF;
	data[5] = (velff_scaled >> 8) & 0xFF;

	int16_t torqueff_scaled = (int16_t)(torqueff);
	data[6] = (torqueff_scaled >> 0) & 0xFF;
	data[7] = (torqueff_scaled >> 8) & 0xFF;

	// send data
	if (CAN_Send(handle, CAN_id, len, data, ext, rtr)) {
		//printf(">> Sent set input pos : nodeid = %d, %.3f rev\n", nodeid, pos_rev);
	}
	else {
		printf("Error :: set input pos\n");
	}

	return 0;
}

void forward_kinematics(float theta1, float theta2) {
	// user input theta1, theta2 : deg
	float kin_theta1_deg = theta1 + THETA1_OFFSET_DEG;
	float kin_theta2_deg = theta2 + THETA2_OFFSET_DEG;

	float q1_rad = deg2rad(kin_theta1_deg);
	float q2_rad = deg2rad(kin_theta2_deg);

	// forward kinematics
	float x = L1 * cos(q1_rad) + L2 * cos(q1_rad + q2_rad);
	float y = L1 * sin(q1_rad) + L2 * sin(q1_rad + q2_rad);

	printf("end effector (x, y) : (%.3f, %.3f)\n", x, y);
}

int inverse_kinematics(float target_x, float target_y, float* theta1_deg_out, float* theta2_deg_out, ElbowSolution solution) {
	// is reachable
	// Max reach : l1 + l2 | Min reach : |l1 - l2|
	float distance = sqrtf(target_x * target_x + target_y * target_y); // distance to target
	if (distance > (L1 + L2)) {
		printf("Error :: Target point is too far (distance: %.3f, max reach: %.3f)\n", distance, L1 + L2);
		return 0;
	}
	if (distance < fabs(L1 - L2)) {
		printf("Error :: Target point is too close (distance: %.3f, min reach: %.3f)\n", distance, fabs(L1 - L2));
		return 0;
	}

	// theta2 cos value
	float cos_q2 = (target_x * target_x + target_y * target_y - L1 * L1 - L2 * L2) / (2 * L1 * L2);

	// refine
	if (cos_q2 > 1.0)
		cos_q2 = 1.0;
	if (cos_q2 < -1.0)
		cos_q2 = -1.0;

	// elbow up, elbow down
	float q2_up_rad = acos(cos_q2);
	float q2_down_rad = -acos(cos_q2);

	float k1, k2, q1_up_rad, q1_down_rad;

	// Sol1 : Elbow-up
	k1 = L1 + L2 * cos(q2_up_rad);
	k2 = L2 * sin(q2_up_rad);
	q1_up_rad = atan2(target_y, target_x) - atan2(k2, k1);

	// Sol2 : Elbow-down
	k1 = L1 + L2 * cos(q2_down_rad);
	k2 = L2 * sin(q2_down_rad);
	q1_down_rad = atan2(target_y, target_x) - atan2(k2, k1);

	// normalize angle (pi ~ -pi)
	auto normalizeAngle = [](float angle) {
		while (angle > M_PI) angle -= 2 * M_PI;
		while (angle < -M_PI) angle += 2 * M_PI;
		return angle;
		};
	q1_up_rad = normalizeAngle(q1_up_rad);
	q2_up_rad = normalizeAngle(q2_up_rad);
	q1_down_rad = normalizeAngle(q1_down_rad);
	q2_down_rad = normalizeAngle(q2_down_rad);

	// select elbow up & down
	 float result_theta1_rad, result_theta2_rad;

	 switch (solution)
	 {
	 case ElbowSolution::SOLUTION_AUTO:
	 default:
		 if (target_y >= 0.0f) {
			 // Elbow-down
			 result_theta1_rad = q1_down_rad;
			 result_theta2_rad = q2_down_rad;
			 //printf("Using ELBOW-DOWN solution.\n");
		 }
		 else {
			 // Elbow-up
			 result_theta1_rad = q1_up_rad;
			 result_theta2_rad = q2_up_rad;
			 //printf("Using ELBOW-UP solution.\n");
		 }
		 break;

	 case ElbowSolution::SOLUTION_ELBOW_UP:
		 result_theta1_rad = q1_up_rad;
		 result_theta2_rad = q2_up_rad;
		 printf("Forcing Elbow-Up solution.\n");
		 break;

	 case ElbowSolution::SOLUTION_ELBOW_DOWN:
		 result_theta1_rad = q1_down_rad;
		 result_theta2_rad = q2_down_rad;
		 printf("Forcing Elbow-Down solution.\n");
		 break;
	 }

	float result_theta1_deg = rad2deg(result_theta1_rad);
	float result_theta2_deg = rad2deg(result_theta2_rad);

	*theta1_deg_out = result_theta1_deg - THETA1_OFFSET_DEG;
	*theta2_deg_out = result_theta2_deg - THETA2_OFFSET_DEG;

	return 1;
}

void cubic_trajectory_fk_one_link(int node, float final_deg, float duration_sec)
{
	float initial_deg = get_motor_angle(node);

	// initial vel, final vel (now, set 0)
	float v0 = 0.0f, vf = 0.0f;		// 1st link

	// Cubic Trajectory = a0 + a1*t a2*t^2 + a3*t^3
	float T = duration_sec;
	// coefficient
	float a0 = initial_deg;
	float a1 = v0;
	float a2 = (3 * (final_deg - initial_deg) - (2 * v0 + vf) * T) / (T * T);
	float a3 = (-2 * (final_deg - initial_deg) + (v0 + vf) * T) / (T * T * T);

	// step
	float dt = 0.02f; // 0.02 sec
	int steps = (int)(T / dt);

	if (steps < 10) steps = 10;

	for (int i = 0; i <= steps; ++i) {
		float t = (float)i * dt;

		float theta = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
		float vel = fabs(a1 + 2 * a2 * t + 3 * a3 * t * t);
		float torque = 0.1f;

		set_input_pos(node, theta, vel, torque);

		Sleep(20);
	}
}

void cubic_trajectory_fk_two_links(float final_deg1, float final_deg2, float duration_sec)
{
	float initial_deg1 = get_motor_angle(1);
	float initial_deg2 = get_motor_angle(2);

	// initial vel, final vel (now, set 0)
	float v0_1 = 0.0f, vf_1 = 0.0f;		// 1st link
	float v0_2 = 0.0f, vf_2 = 0.0f;		// 2nd link

	// Cubic Trajectory = a0 + a1*t a2*t^2 + a3*t^3
	float T = duration_sec;
	// coefficient
	float a0_1 = initial_deg1;
	float a1_1 = v0_1;
	float a2_1 = (3 * (final_deg1 - initial_deg1) - (2 * v0_1 + vf_1) * T) / (T * T);
	float a3_1 = (-2 * (final_deg1 - initial_deg1) + (v0_1 + vf_1) * T) / (T * T * T);

	float a0_2 = initial_deg2;
	float a1_2 = v0_2;
	float a2_2 = (3 * (final_deg2 - initial_deg2) - (2 * v0_2 + vf_2) * T) / (T * T);
	float a3_2 = (-2 * (final_deg2 - initial_deg2) + (v0_2 + vf_2) * T) / (T * T * T);

	// step
	float dt = 0.02f; // 0.02 sec
	int steps = (int)(T / dt);

	if (steps < 10) steps = 10;

	for (int i = 0; i <= steps; ++i) {
		float t = (float)i * dt;

		float theta1 = a0_1 + a1_1 * t + a2_1 * t * t + a3_1 * t * t * t;
		float theta2 = a0_2 + a1_2 * t + a2_2 * t * t + a3_2 * t * t * t;

		float vel1 = fabs(a1_1 + 2 * a2_1 * t + 3 * a3_1 * t * t);
		float vel2 = fabs(a1_2 + 2 * a2_2 * t + 3 * a3_2 * t * t);

		float torque = 0.1f;

		set_input_pos(1, theta1, vel1, torque);
		set_input_pos(2, theta2, vel2, torque);

		Sleep(20);
	}
}

void cubic_trajectory_ik_two_links(float end_x, float end_y, float duration_sec_link1, float duration_sec_link2, ElbowSolution solution)
{
	float initial_theta1 = get_motor_angle(1);
	float initial_theta2 = get_motor_angle(2);

	float final_theta1, final_theta2;
	if (!inverse_kinematics(end_x, end_y, &final_theta1, &final_theta2, solution)) {
		printf(">> IK failed at target (%.3f, %.3f)\n", end_x, end_y);
		return;
	}

	// initial vel, final vel (now, set 0)
	float v0_1 = 0.0f, vf_1 = 0.0f;
	float v0_2 = 0.0f, vf_2 = 0.0f;

	// Cubic Trajectory coefficients (각자 duration 사용)
	float T1 = duration_sec_link1;
	float T2 = duration_sec_link2;

	// Link1
	float a0_1 = initial_theta1;
	float a1_1 = v0_1;
	float a2_1 = (3 * (final_theta1 - initial_theta1) - (2 * v0_1 + vf_1) * T1) / (T1 * T1);
	float a3_1 = (-2 * (final_theta1 - initial_theta1) + (v0_1 + vf_1) * T1) / (T1 * T1 * T1);

	// Link2
	float a0_2 = initial_theta2;
	float a1_2 = v0_2;
	float a2_2 = (3 * (final_theta2 - initial_theta2) - (2 * v0_2 + vf_2) * T2) / (T2 * T2);
	float a3_2 = (-2 * (final_theta2 - initial_theta2) + (v0_2 + vf_2) * T2) / (T2 * T2 * T2);

	// step
	float dt = 0.02f; // 0.02 sec
	int steps = (int)(fmaxf(T1, T2) / dt);
	if (steps < 10) steps = 10;

	for (int i = 0; i <= steps; ++i) {
		float t = (float)i * dt;

		// Link1 보간 (T1 초과 시 마지막 값 유지)
		float theta1, vel1;
		if (t <= T1) {
			theta1 = a0_1 + a1_1 * t + a2_1 * t * t + a3_1 * t * t * t;
			vel1 = fabs(a1_1 + 2 * a2_1 * t + 3 * a3_1 * t * t);
		}
		else {
			theta1 = final_theta1;
			vel1 = 0.0f;
		}

		// Link2 보간 (T2 초과 시 마지막 값 유지)
		float theta2, vel2;
		if (t <= T2) {
			theta2 = a0_2 + a1_2 * t + a2_2 * t * t + a3_2 * t * t * t;
			vel2 = fabs(a1_2 + 2 * a2_2 * t + 3 * a3_2 * t * t);
		}
		else {
			theta2 = final_theta2;
			vel2 = 0.0f;
		}

		float torque = 0.1f;

		set_input_pos(1, theta1, vel1, torque);
		set_input_pos(2, theta2, vel2, torque);

		Sleep(20);
	}
}

float lerp(float start, float end, float t)
{
	// linear interpolation
	return start + (end - start) * t;
}

void moveL(float start_x, float start_y, float end_x, float end_y, float duration_sec, ElbowSolution solution) {

	float current_x, current_y;
	float theta1, theta2;
	float vel = 0.1f, torque = 0.1f;

	// step
	float dt = 0.02f; // 0.02 sec
	int steps = (int)(duration_sec / dt);

	if (steps < 10) steps = 10;

	for (int i = 0; i <= steps; i++) {
		float t = (float)i / steps;	// 0~1

		// cubic trajectory
		float s = -2 * t * t * t + 3 * t * t;

		current_x = lerp(start_x, end_x, s);
		current_y = lerp(start_y, end_y, s);

		inverse_kinematics(current_x, current_y, &theta1, &theta2, solution);

		set_input_pos(1, theta1, vel, torque);
		set_input_pos(2, theta2, vel, torque);

		Sleep(20);
	}
}
