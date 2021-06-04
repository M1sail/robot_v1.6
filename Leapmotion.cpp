#include "stdafx.h"
#include "robot.h"  

void myLeapmotion::getFrame() {
    float leapdata[3][3];

    const Frame frame = controller.frame();

    HandList hands = frame.hands();

    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
        const Hand hand = *hl;

        leapdata[0][0] = hand.fingers().extended().count();

        for (int i = 0; i < 3; i++) { leapdata[1][i] = hand.palmPosition()[i]; }

         Vector normal = hand.palmNormal();
         Vector direction = hand.direction();

        leapdata[2][0] = -direction.yaw();  leapdata[2][1] = - normal.roll();  leapdata[2][2] = direction.pitch();

        FingerList fingers = hand.fingers();
        leapdata[0][1] = sqrt(pow(fingers[0].tipPosition()[0] - fingers[1].tipPosition()[0], 2) + pow(fingers[0].tipPosition()[1] - fingers[1].tipPosition()[1], 2) + pow(fingers[0].tipPosition()[2] - fingers[1].tipPosition()[2], 2));

        if (hand.isLeft())
        {
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    leapdata_L[i][j] = leapdata[i][j];
            leapdata_L[2][1] += 1.57;
        }
        else
        {
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    leapdata_R[i][j] = leapdata[i][j];
            leapdata_R[2][1] -= 1.57;
        }
    }

    float p_tem0[] = { 0, 0, 0 };						//获取末端位置
    float p_tem1[] = { 0, 0, 0 };
    for (int i = 0; i < 3; i++)
        p_tem0[i] = leapdata_R[1][i];
    matrix_multiply_3x1(p_tem1, rot, p_tem0);
    for (int i = 0; i < 3; i++)
        data[0][i] = p_tem1[i] + p_move[i];

    for (int i = 0; i < 3; i++)
        p_tem0[i] = leapdata_L[1][i];
    matrix_multiply_3x1(p_tem1, rot, p_tem0);
    for (int i = 0; i < 3; i++)
        data[2][i] = p_tem1[i] + p_move[i];

    for (int i = 0; i < 3; i++)						//获取末端姿态
    {
        data[1][i] = leapdata_R[2][i];
        data[3][i] = leapdata_L[2][i];
    }

    if (leapdata_R[0][1] < 30) data[4][0] = 10 * PI / 180;                       //夹子
    else if (30 <= leapdata_R[0][1] && leapdata_R[0][1] < 90) data[4][0] = 60 * PI / 180;
    else if (leapdata_R[0][1] >= 90) data[4][0] = 80 * PI / 180;

    if (leapdata_L[0][1] < 30)  data[4][1] = 10 * PI / 180;
    else if (30 <= leapdata_L[0][1] && leapdata_L[0][1] < 90) data[4][1] = 60 * PI / 180;
    else if (leapdata_L[0][1] >= 90) data[4][1] = 80 * PI / 180;

    if (!frame.hands().isEmpty()) {
        std::cout << std::endl;
    }
}

void myLeapmotion::Init(const Controller& controller) {
    std::cout << "Initialized" << std::endl;
}

void myLeapmotion::Connect(const Controller& controller) {
    std::cout << "Connected" << std::endl;
}

void myLeapmotion::Disconnect(const Controller& controller) {
    // Note: not dispatched when running in a debugger.
    std::cout << "Disconnected" << std::endl;
}

void myLeapmotion::Exit(const Controller& controller) {
    std::cout << "Exited" << std::endl;
}

void myLeapmotion::ini_leap() {
    Init(controller);
    Connect(controller);
    Disconnect(controller);
    Exit(controller);
}