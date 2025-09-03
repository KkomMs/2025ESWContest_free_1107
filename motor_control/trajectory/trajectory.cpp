#include "combined_arduinos.h"
#include "motor_controller.h"
#include "combined_dxl.h"

#define NOMINMAX
#define QUEUE_SIZE 10   // order queue max size
#include <windows.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// 운영 중 초기 위치 세팅을 1회만 수행하기 위한 플래그
bool g_init_done = false;

// ===== ROBOT PARAMETER ===== //
const float THETA2_OFFSET_DEG = 25.0f;  // (CW) initial theta for joint2

// void fnction pointer
typedef void (*element)(void);
// circle queue struct
typedef struct __circleQueue{
    int rear;
    int front;
    element* data;
}Queue;
Queue orders;       // coffee order queue

enum class IceMaker {
    MAKE_ICE_AMERICANO,
    MAKE_HOT_AMERICANO,
    MAKE_ICE_TEA,
    MAKE_ICE_TEA_AMERICANO
};

void init_queue(Queue* q) {
    q->front = 0;
    q->rear = 0;
    q->data = (element*)malloc(sizeof(element) * QUEUE_SIZE);
}

bool is_full(Queue* q) {
    if (((q->rear + 1) % QUEUE_SIZE) == q->front)
        return true;
    else
        return false;
}

bool is_empty(Queue* q) {
    if (q->front == q->rear)
        return true;
    else
        return false;
}

void enqueue(Queue* q, element data) {
    if (is_full(q)) {
        printf("Queue is full.\n");
        return;
    }
    else {
        q->rear = (q->rear + 1) % (QUEUE_SIZE);
        q->data[q->rear] = data;
    }
    return;
}

element dequeue(Queue* q) {
    if (is_empty(q)) {
        printf("Queue is empty.\n");
        return NULL;
    }
    else {
        q->front = (q->front + 1) % (QUEUE_SIZE);
        return q->data[q->front];
    }
}

bool get_cup();
bool move_to_ice_maker(IceMaker beverage);
bool get_coffee();
bool get_syrup();
const float it_syrup = 400.0f;
const float ita_syrup = 250.0f;

void ice_americano() {

    // ========== making ice americano sequence ========== //
    Sleep(1000);
    get_cup();  // move to cup dispenser, get cup

    move_to_ice_maker(IceMaker::MAKE_ICE_AMERICANO);    // move to ice maker, get ice and water

    get_coffee();   // move to coffee machine, get espresso

    // ===== move to take-out zone =====
    sendToLinear(2, 0, 80);        // linear motor down
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, 110, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, (-125.0f - THETA2_OFFSET_DEG), 2.0f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -110, 3.0f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, (-41.0f - THETA2_OFFSET_DEG), 2.0f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -130, 1.5f);
    Sleep(1000);
    sendToLinear(2, 0, 20);
    Sleep(3000);
    moveGripper(-260, 4.3f, -1);        // open gripper
    Sleep(3000);
    moveL(-5.585f, -4.865f, -3.0f, -4.865f, 4.0f, ElbowSolution::SOLUTION_ELBOW_DOWN);
    Sleep(1000);
    moveGripper(260, 2.0f, 1);        // close gripper
    Sleep(1000);
    sendToLinear(2, 0, 30);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -120, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, -60, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -130, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, -30, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -145, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, 0, 1.5f);
    Sleep(1000);
}

void hot_americano() {

    // ========== making hot americano sequence ========== //
    Sleep(1000);

    get_cup();

    move_to_ice_maker(IceMaker::MAKE_HOT_AMERICANO);    // water

    get_coffee();

    // ===== move to take-out zone =====
    sendToLinear(2, 0, 80);        // linear motor down
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, 110, 1.0f);       
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, (-125.0f - THETA2_OFFSET_DEG), 2.0f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -127, 3.0f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, (-87.0f - THETA2_OFFSET_DEG), 2.0f);
    Sleep(1000);
    sendToLinear(2, 0, 20);
    Sleep(3000);
    moveGripper(-260, 6.3f, -1);        // open gripper
    Sleep(3000);
    moveL(-5.502f, -1.717f, -3.f, -1.717f, 4.0f, ElbowSolution::SOLUTION_ELBOW_DOWN);
    Sleep(1000);
    moveGripper(260, 4.0f, 1);        // close gripper
    Sleep(1000);
    sendToLinear(2, 0, 30);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, -80, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -120, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, -60, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -130, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, -30, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(1, -145, 1.5f);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, 0, 1.5f);
    Sleep(1000);
}

void ice_tea() {
    // ========== making ice americano sequence ========== //
    Sleep(1000);
    get_cup();

    move_to_ice_maker(IceMaker::MAKE_ICE_TEA);  // ice, water

    get_syrup(it_syrup);
  
    // ===== move to take-out zone =====
    sendToLinear(2, 1, 45);        // linear motor down
    Sleep(1000);
    cubic_trajectory_fk_two_links(98.0f, (90.0f - THETA2_OFFSET_DEG), 2.5f);       // take-out zone
    Sleep(1500);
    sendToLinear(2, 0, 20);
    Sleep(3000);
    moveGripper(-260, 6.3f, -1);        // open gripper
    Sleep(3000);
    moveL(-5.391f, 1.597f, -3.0f, 1.597f, 4.0f, ElbowSolution::SOLUTION_ELBOW_UP);
    Sleep(1000);
    moveGripper(260, 4.0f, 1);  // close gripper
    Sleep(1000);
    sendToLinear(2, 0, 35);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, (180.0f - THETA2_OFFSET_DEG), 1.5f);
    Sleep(500);
}

void ice_tea_americano() {
    // ========== making ice americano sequence ========== //
    Sleep(1000);
    get_cup();

    move_to_ice_maker(IceMaker::MAKE_ICE_TEA_AMERICANO);    // ice, water

    get_syrup(ita_syrup);

    // ==== = move to coffee machine ======
    sendToLinear(2, 1, 125);        // linear motor up
    Sleep(500);

    get_coffee();

    // ===== move to take-out zone =====
    sendToLinear(2, 0, 80);        // linear motor down
    Sleep(1000);
    cubic_trajectory_fk_two_links(103.0f, (40.0f - THETA2_OFFSET_DEG), 2.5f);       // take-out zone
    Sleep(1500);
    sendToLinear(2, 0, 20);
    Sleep(2000);
    moveGripper(-260, 4.3f, -1);        // open gripper
    Sleep(3000);
    moveL(-5.639f, 4.838f, -3.0f, 4.838f, 4.0f, ElbowSolution::SOLUTION_ELBOW_UP);
    Sleep(1000);
    moveGripper(260, 2.0f, 1);   // close gripper
    Sleep(1000);
    sendToLinear(2, 0, 30);
    Sleep(1000);
    cubic_trajectory_fk_one_link(2, (180.0f - THETA2_OFFSET_DEG), 1.5f);
    Sleep(500);
}

void order(int *select, int *cups) {
    printf("\n음료를 선택하세요. (1: Ice Americano || 2: Hot Americano || 3: Ice Tea || 4: Ice Tea + Americano): ");
    scanf_s(" %d", select);

    printf("몇 잔 만들까요? (정수 입력): ");
    scanf_s(" %d", cups);

    for (int i = 0; i < *cups; i++) {
        if (*select == 1) {
            enqueue(&orders, ice_americano);
        }
        else if (*select == 2) {
            enqueue(&orders, hot_americano);
        }
        else if (*select == 3) {
            enqueue(&orders, ice_tea);
        }
        else if (*select == 4) {
            enqueue(&orders, ice_tea_americano);
        }
    }
}

// ================================
// 메인: 무한 루프 + 잔수 입력 + q로 종료
// ================================
int main() {
    initArduinos();                     // set arduinos
    setupDXL();                        // set gripper, dispenser dxl
    sendToLinear(2, 0, 0);             // Linear motor initialization
    setManipulatorControl();           // robot arm initialization
    cubic_trajectory_fk_two_links(-38.0f, (0-THETA2_OFFSET_DEG), 0.7f);    // initial position
    printf("\nSet initial settings\n");

    int check;
    int select;
    int cups;
    int sup;

    init_queue(&orders);

    while (true) {
        bool flag = true;
        printf("\n주문하시겠습니까? (1: Yes | 2: No): ");
        scanf_s(" %d", &check);

        if (check == 2)
            break;

        while (flag) {
            order(&select, &cups);
            printf("추가 주문하시겠습니까? (1: Yes | 2: No): ");
            scanf_s(" %d", &sup);

            if (sup == 2)
                flag = false;
        }
        printf("\n주문이 완료되었습니다. 음료 제작을 시작합니다.\n");
        cubic_trajectory_fk_one_link(2, (180.0f - THETA2_OFFSET_DEG), 1.5f);
        Sleep(1000);

        while (!is_empty(&orders)) {
            element make_coffee = dequeue(&orders);     // save the function pointer
            if (make_coffee != NULL)
                make_coffee();      // function execution
        }
        printf("모든 음료 제작이 완료되었습니다. 초기 위치로 복귀합니다.\n");
        Sleep(5000);
        cubic_trajectory_fk_one_link(2, (180.0f-THETA2_OFFSET_DEG), 1.5f);
        Sleep(500);
        cubic_trajectory_fk_one_link(1, -38.0f, 2);
    }
    free(orders.data);

    closeDispenser();
    closeGripper();
    closeSerials();
    CAN_close();

    return 0;
}

bool get_cup() {
    printf("Getting cup from dispenser...\n");
    cubic_trajectory_fk_one_link(1, -154.608f, 2.0f);
    Sleep(500);
    cubic_trajectory_fk_one_link(2, (62.807f - THETA2_OFFSET_DEG), 1.2f);
    Sleep(500);
    moveDispenser(-70, 0.5f, -1);
    Sleep(500);
    moveGripper(260, 2.3f, 1);
    Sleep(500);
    sendToLinear(2, 1, 15);
    Sleep(500);
    moveL(-1.775f, -6.522f, -1.775f, -5.0f, 1.5f);
    Sleep(500);
    sendToLinear(2, 0, 20);
    Sleep(500);

    return true;
}

bool move_to_ice_maker(IceMaker beverage) {
    switch (beverage) {
    case IceMaker::MAKE_ICE_AMERICANO:
        cubic_trajectory_fk_one_link(1, -150.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_two_links(-112.5f, (50.5f - THETA2_OFFSET_DEG), 1.5f);   // ice
        Sleep(500);
        sendToicemaker(4, 3);
        Sleep(15000);

        cubic_trajectory_fk_one_link(1, -132.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_one_link(2, (95.0f - THETA2_OFFSET_DEG), 1);         // water
        Sleep(500);
        sendToicemaker(3, 4);
        Sleep(20000);

        cubic_trajectory_fk_one_link(1, -150.0f, 0.8f);
        Sleep(500);
        cubic_trajectory_fk_one_link(2, (119.0f - THETA2_OFFSET_DEG), 0.8f);
        Sleep(500);

        sendToLinear(2, 1, 125);        // linear motor up
        Sleep(500);
        break;
    case IceMaker::MAKE_HOT_AMERICANO:
        cubic_trajectory_fk_one_link(1, -150.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_one_link(2, (80.0f - THETA2_OFFSET_DEG), 1);
        Sleep(500);
        cubic_trajectory_fk_one_link(1, -132.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_one_link(2, (95.0f - THETA2_OFFSET_DEG), 1);         // water
        Sleep(500);
        sendToicemaker(3, 10);
        Sleep(20000);

        cubic_trajectory_fk_one_link(1, -150.0f, 0.8f);
        Sleep(500);
        cubic_trajectory_fk_one_link(2, (119.0f - THETA2_OFFSET_DEG), 0.8f);
        Sleep(500);

        sendToLinear(2, 1, 120);        // linear motor up
        Sleep(500);
        break;
    case IceMaker::MAKE_ICE_TEA:
        cubic_trajectory_fk_one_link(1, -150.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_two_links(-112.5f, (50.5f - THETA2_OFFSET_DEG), 1.5f);   // ice
        Sleep(500);
        sendToicemaker(4, 3);
        Sleep(15000);

        cubic_trajectory_fk_one_link(1, -132.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_one_link(2, (95.0f - THETA2_OFFSET_DEG), 1);         // water
        Sleep(500);
        sendToicemaker(3, 7);
        Sleep(20000);
        break;
    case IceMaker::MAKE_ICE_TEA_AMERICANO:
        cubic_trajectory_fk_one_link(1, -150.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_two_links(-112.5f, (50.5f - THETA2_OFFSET_DEG), 1.5f);   // ice
        Sleep(500);
        sendToicemaker(4, 3);
        Sleep(15000);

        cubic_trajectory_fk_one_link(1, -132.0f, 1);
        Sleep(500);
        cubic_trajectory_fk_one_link(2, (95.0f - THETA2_OFFSET_DEG), 1);         // water
        Sleep(500);
        sendToicemaker(3, 3);
        Sleep(20000);
        break;
    }

    return true;
}

bool get_coffee() {
    cubic_trajectory_fk_one_link(1, 90, 1.5f);
    Sleep(500);
    cubic_trajectory_fk_one_link(2, 0, 1.5f);
    Sleep(500);
    cubic_trajectory_fk_one_link(1, 105, 1.5f);
    Sleep(500);
    cubic_trajectory_fk_one_link(2, (-42.5f - THETA2_OFFSET_DEG), 2.0f);        // coffee machine
    Sleep(500);
    cubic_trajectory_fk_one_link(1, 71.5f, 2.0f);
    Sleep(3000);

    sendTocoffeemachine(2);     // espresso
    Sleep(50000);

    moveL(3.493f, 6.490f, 1.693f, 6.490f, 2.2f);  // draw out cup
    Sleep(1000);

    return true;
}

bool get_syrup(float beverage) {
    cubic_trajectory_fk_one_link(1, -150.0f, 0.8f);
    Sleep(500);
    cubic_trajectory_fk_one_link(2, (119.0f - THETA2_OFFSET_DEG), 0.8f);
    Sleep(500);
    cubic_trajectory_fk_one_link(1, -15.0f, 1.5f);
    Sleep(500);
    cubic_trajectory_fk_one_link(2, (70.0f - THETA2_OFFSET_DEG), 1.5f);
    Sleep(500);
    cubic_trajectory_fk_one_link(1, -12.0f, 1.5f);
    Sleep(500);
    cubic_trajectory_fk_one_link(2, (62.0f - THETA2_OFFSET_DEG), 1.5f);
    Sleep(500);
    sendToLinear(1, 0, beverage);                                     //syrup 
    Sleep(20000);
    cubic_trajectory_fk_one_link(2, (70.0f - THETA2_OFFSET_DEG), 0.8f);
    Sleep(500);
    cubic_trajectory_fk_one_link(1, -15.0f, 0.8f);
    Sleep(500);
    cubic_trajectory_fk_one_link(2, (119.0f - THETA2_OFFSET_DEG), 0.8f);

    return true;
}
