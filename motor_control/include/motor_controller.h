#pragma once
#include <stdint.h>

enum class ElbowSolution {
	SOLUTION_AUTO,
	SOLUTION_ELBOW_UP,
	SOLUTION_ELBOW_DOWN
};

int CAN_close();
int set_input_pos(uint8_t nodeid, float pos_degree, float velff, float torqueff);
void print_encoder_pos(uint8_t nodeid);
float get_motor_angle(int nodeid);
void print_current_motor_angle(int nodeid);
void setManipulatorControl();
void forward_kinematics(float theta1, float theta2);
int inverse_kinematics(float target_x, float target_y, float* theta1_deg_out, float* theta2_deg_out, ElbowSolution solution = ElbowSolution::SOLUTION_AUTO);
void cubic_trajectory_fk_one_link(int node, float final_deg, float duration_sec);
void cubic_trajectory_fk_two_links(float final_deg1, float final_deg2, float duration_sec);
void cubic_trajectory_ik_two_links(float end_x, float end_y, float duration_sec_link1, float duration_sec_link2, ElbowSolution solution = ElbowSolution::SOLUTION_AUTO);
void moveL(float start_x, float start_y, float end_x, float end_y, float duration_sec, ElbowSolution solution = ElbowSolution::SOLUTION_AUTO);
