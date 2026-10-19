/*--------------------------------NO.7----------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       tooy0                                                     */
/*    Created:      9/2/2026, 9:47:19 AM                                      */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <math.h>

using namespace vex;

brain Brain;
inertial BrainInertial = inertial();

// AI Classification Competition Element IDs - Level Up
enum gameElementsLevelUp
{
    blueBeanBag,
    redBeanBag,
    yellowBeanBag
};

controller Controller;
motor Catapult7(PORT1, true);
motor ForwardLeft3(PORT3, true);
motor RearLeft6(PORT6, false);
motor_group DT_Left(ForwardLeft3, RearLeft6);
motor ForwardRight9(PORT9, false);
motor RearRight12(PORT12, true);
motor_group DT_Right(ForwardRight9, RearRight12);
motor Intake1(PORT7);
pneumatic Pneumatic4(PORT4);
pneumatic Pneumatic10(PORT10);
touchled Touchled8(PORT8);
colorsensor Color2(PORT2);
distance distance5(PORT5);
colorsensor Color11(PORT11);

// AI Vision Color Descriptions
// aivision::colordesc COLORR(1, 194, 14, 9, 21, 0.7);
aivision::colordesc COLORR(1, 213, 17, 20, 13, 0.48);
aivision::colordesc COLORY(2, 193, 174, 67, 11, 0.25);
// aivision::colordesc COLORB(3, 34, 134, 200, 15, 0.23);
aivision::colordesc COLORB(2, 33, 148, 221, 20, 0.25);
// AI Vision Code Descriptions
vex::aivision AIVision5(PORT5, COLORR, COLORY, COLORB);

void Catareload();
// User defined function
void SetVel_DtLeft_DtRight(double SetVel_DtLeft_DtRight__DtLeft, double SetVel_DtLeft_DtRight__DtRight);
// User defined function
void DriveTrainStop(brakeType mode);
// User defined function
void Intake();
// AI Cam
// int findBeamBag();
int findBeamBag2();
// shot
void Shot();
// cal
int CalibrateInertial();
// Intake and catareload
int Unnecessary_step();
// arm up and down
void UpandDown();
// ArmUp
void ArmUp();

int closestIndex = 0;
int maxY = -1;

float SpeedLeft, SpeedRight, Obj_Now, Obj_after;
float count_Bag = 0.0;

bool IntakeReload, reload, beambag, reloadcomplete, Forword, Reverse, RedBlue, start, InTakeUp = false;

bool Up, Arm_IntakeReload, Flip = true;

int waitTimeforshoot = 1500;

task *autoL3 = nullptr;

void ChackArm_Up_Down()
{
    if (Arm_IntakeReload)
    {
        UpandDown();
        Touchled8.setColor(red);
    }
    else
    {
        Touchled8.setColor(green);
    }
}

void UpandDown()
{
    if (Up)
    {
        // =============== L4 shoot Up ====================
        Pneumatic10.extend(cylinder2);
    }
    else
    {
        // =============== L4 shoot Down ====================
        Pneumatic10.retract(cylinder2);
    }
    Up = !Up;
}

double ComputeTurnSpeed(double absErr, double maxSpeed)
{
    double minSpeed = maxSpeed * 0.2;
    double maxErrForScale = 95.0;

    double t = absErr / maxErrForScale;
    if (t > 1.0)
        t = 1.0;

    // curve: make small errors give much smaller speeds
    t = t * t; // square makes the curve “steeper” near 0

    double speed = minSpeed + (maxSpeed - minSpeed) * t;
    return speed;
}

void TurnRight_h_v_m(double heading, double velocity, double momentum)
{
    while (true)
    {
        double current = BrainInertial.heading();
        double error = heading - current; // positive → need left turn
        double absErr = fabs(error);

        // ---- 1. Done / inside momentum zone ----
        if (absErr <= momentum)
        {
            DT_Left.setStopping(brake);
            DT_Right.setStopping(brake);
            DT_Left.stop();
            DT_Right.stop();

            while (DT_Left.velocity(vex::velocityUnits::pct) > 1)
                wait(10, msec);

            printf("Final heading: %.2f\n", current);
            break;
        }

        // ---- 2. Slowdown system to prevent sliding ----
        double turnSpeed = ComputeTurnSpeed(absErr, velocity);
        // turnSpeed *= 0.9;
        if (turnSpeed < 10)
            turnSpeed = 10;

        DT_Left.setVelocity(turnSpeed, percent);
        DT_Right.setVelocity(turnSpeed, percent);

        // ---- 3. Actual turn left ----
        DT_Left.spin(forward);
        DT_Right.spin(reverse);

        wait(20, msec);
    }

    wait(0.1, seconds);
}

void TurnLeft_h_v_m(double heading, double velocity, double momentum)
{
    while (true)
    {
        double current = BrainInertial.heading();
        double error = heading - current; // negative → need right turn
        double absErr = fabs(error);

        // ---- 1. Done / inside momentum zone ----
        if (absErr <= momentum)
        {
            DT_Left.setStopping(brake);
            DT_Right.setStopping(brake);
            DT_Left.stop();
            DT_Right.stop();

            while (DT_Left.velocity(vex::velocityUnits::pct) > 1)
                wait(10, msec);

            printf("Final heading: %.2f\n", current);
            break;
        }

        // ---- 2. Slowdown system to prevent sliding ----
        double turnSpeed = ComputeTurnSpeed(absErr, velocity);
        // turnSpeed *= 0.9;
        // if(turnSpeed < 10) turnSpeed = 10;

        DT_Left.setVelocity(turnSpeed, percent);
        DT_Right.setVelocity(turnSpeed, percent);

        // ---- 3. Actual turn right ----
        DT_Left.spin(reverse);
        DT_Right.spin(forward);

        wait(20, msec);
    }

    wait(0.1, seconds);
}

// User defined function
void Catareload()
{
    Catapult7.setVelocity(127.0, percent);
    Catapult7.setMaxTorque(100.0, percent);
    Catapult7.setStopping(brake);
    Catapult7.spin(forward);
    wait(450, msec);
    reload = false;
    while (fabs(Catapult7.velocity(pct)) > 1)
    {
        /* code */
        task::sleep(20);
    }
    task::sleep(80);
    Catapult7.stop();
}
//============================= setDT funtion =============================

// setvelocity of left and right drive train motors
void SetVel_DtLeft_DtRight(double LeftVel, double RightVel)
{
    DT_Left.setVelocity(LeftVel, percent);
    DT_Right.setVelocity(RightVel, percent);
}

// setstopping mode of left and right drive train motors
void DriveTrainStop(brakeType mode)
{
    DT_Left.setStopping(mode);
    DT_Right.setStopping(mode);
    DT_Left.stop();
    DT_Right.stop();
}

void IntakeUp()
{
    if (InTakeUp)
    {
        InTakeUp = false;
        Intake1.setVelocity(127.0, percent);
        Intake1.setMaxTorque(100.0, percent);
        Intake1.setStopping(coast);
        Intake1.spin(forward);
        wait(0.6, seconds);
        Intake1.setStopping(coast);
        Intake1.stop();
    }
    else
    {
        InTakeUp = true;
        Intake1.setVelocity(127.0, percent);
        Intake1.setMaxTorque(100.0, percent);
        Intake1.setStopping(brake);
        Intake1.spin(reverse);
        while (fabs(Intake1.position(degrees)) > 10)
        {
            /* ___ */
            task::sleep(20);
        }
        Intake1.setStopping(brake);
        Intake1.stop();
    }
}
// User defined function
void Intake()
{
    Pneumatic10.extend(cylinder1);
    wait(0.2, seconds);

    Intake1.setVelocity(100, percent);
    Intake1.setMaxTorque(100, percent);
    // Intake1.setStopping(coast);
    Intake1.spin(forward);

    int waitTime = 0;

    // while (fabs(Intake1.position(degrees)) < 40 &&
    //        waitTime < 1500)
    // {
    //     wait(20, msec);
    //     waitTime += 20;
    // }
    Intake1.spin(forward);
    // waitTime = 0;
    wait(0.2, seconds);
    while (fabs(Intake1.velocity(pct)) > 5 &&
           waitTime < waitTimeforshoot)
    {
        wait(20, msec);
        waitTime += 20;
    }
    // wait(0.1, seconds);

    Pneumatic10.retract(cylinder1);
    // Intake1.setVelocity(127, percent);
    // Intake1.setMaxTorque(100, percent);
    wait(0.2, seconds);
    Intake1.spin(reverse);

    wait(0.2, seconds);

    waitTime = 0;

    while (fabs(Intake1.position(degrees)) > 40 &&
           waitTime < 1500)
    {
        wait(20, msec);
        waitTime += 20;
    }

    while (fabs(Intake1.velocity(pct)) > 5 &&
           waitTime < 1000)
    {
        wait(20, msec);
        waitTime += 20;
    }

    Intake1.setMaxTorque(5, percent);
    Intake1.setStopping(hold);
    wait(50, msec);
    Intake1.stop();
}

// "when started" hat block
int Start()
{
    DT_Left.setVelocity(100.0, percent);
    DT_Right.setVelocity(100.0, percent);
    Catapult7.setVelocity(100.0, percent);
    DT_Left.setMaxTorque(100.0, percent);
    DT_Right.setMaxTorque(100.0, percent);
    Catapult7.setMaxTorque(100.0, percent);
    Intake1.setMaxTorque(100.0, percent);
    Catapult7.setStopping(brake);
    DT_Left.setStopping(brake);
    DT_Right.setStopping(brake);
    SetVel_DtLeft_DtRight(2.0, 2.0);
    DT_Left.spin(forward);
    DT_Right.spin(forward);
    return 0;
}
void L3()
{
    // DT_Right.spinFor(reverse, 100, degrees);
    int count = 0;
    Color2.setLight(100, percent);
    wait(50, msec);
    SetVel_DtLeft_DtRight(80.0, 100.0);
    // ---- Down L4 ---------
    Up = true;
    UpandDown();
    // -----Go shoot L3---------
    DT_Left.spin(reverse);
    DT_Right.spin(reverse);
    wait(0.2, seconds);
    while (Color2.hue() >= 90.0 || fabs(DT_Right.velocity(percent) >= 40.0))
    { // check color yellow check velocity drivetrainRight 10 percent
    }
    // ------- adjust angle ---------
    Pneumatic4.retract(cylinder1);
    // wait(0.2, seconds);
    DT_Left.spin(forward);
    DT_Right.spin(reverse);
    wait(0.2, seconds);
    DT_Left.stop();
    DT_Right.stop();
    // wait(0.1, seconds);
    // --------- shoot --------------
    Shot();
    // --------- Start Exit ---------
    reload = true;
    // Pneumatic4.extend(cylinder2);
    Pneumatic4.extend(cylinder1);
    wait(0.15, seconds);
    SetVel_DtLeft_DtRight(100.0, 100.0);
    DT_Left.spin(forward);
    DT_Right.spin(forward);
    while ((Color2.hue() <= 90.0))
    { // check color yellow check velocity drivetrainRight 10 percent
        if (count == 30)
        {
            SetVel_DtLeft_DtRight(100.0, 100.0);
            DT_Left.spin(reverse);
            wait(100, msec);
            count = 0;
        }
        DT_Left.spin(forward);
        DT_Right.spin(forward);
        wait(20, msec);
        count += 1;
    }
    SetVel_DtLeft_DtRight(20.0, 100.0);
    DT_Left.spin(forward);
    DT_Right.spin(forward);
    wait(0.2, seconds);
}

// void PIDfind(int mode)
// {
//     AIVision5.modelDetection(true);
//     while (true)
//     {
//         AIVision5.takeSnapshot(mode == 0 ? redBeanBag : yellowBeanBag, vex::aivision::objectType::modelObject, 8);
//         if (AIVision5.objectCount == 0)
//         {
//             SetVel_DtLeft_DtRight(10.0, 10.0);
//             DT_Left.spin(forward);
//             DT_Right.spin(forward);
//             wait(20, msec);
//             continue;
//         } // หาอันที่ใกล้สุด
//         int closestIndex = 0;
//         int maxArea = 0;
//         for (int i = 0; i < AIVision5.objectCount; i++)
//         {
//             int area = AIVision5.objects[i].width * AIVision5.objects[i].height;
//             if (area > maxArea)
//             {
//                 maxArea = area;
//                 closestIndex = i;
//             }
//         }
//         int centerX = AIVision5.objects[closestIndex].centerX;
//         if (centerX >= 130 && centerX <= 160)
//         {
//             break;
//         }
//         double turn = 3;
//         if (centerX < 130)
//         {
//             DT_Left.spin(reverse, turn, percent);
//             DT_Right.spin(forward, turn, percent);
//         }
//         if (centerX > 170)
//         {
//             DT_Left.spin(forward, turn, percent);
//             DT_Right.spin(reverse, turn, percent);
//         }
//         // wait(20, msec);
//     }
//     DT_Left.stop(brake);
//     DT_Right.stop(brake);
// }

void ArmUp()
{
    Brain.Timer.reset();
    if (Flip)
    {
        Flip = false;
        Intake1.setVelocity(127.0, percent);
        Intake1.setMaxTorque(100.0, percent);
        Intake1.setStopping(coast);
        Intake1.spin(forward);
        wait(0.6, seconds);
        Intake1.setStopping(coast);
        Intake1.stop();
    }
    else
    {
        Flip = true;
        Intake1.setVelocity(127.0, percent);
        Intake1.setMaxTorque(100.0, percent);
        Intake1.spin(reverse);
        while (fabs(Intake1.position(degrees)) > 10)
        {
            /* ___ */
            task::sleep(20);
        }
        wait(50, msec);
        Intake1.setMaxTorque(5, percent);
        Intake1.setStopping(hold);
        Intake1.stop();
    }
}

// "when Controller ButtonR3 pressed" hat block
void YL4()
{
    start = true;
    BrainInertial.setHeading(0.0, degrees);
    Brain.resetTimer();
    beambag = true;
    reloadcomplete = true;
    //================ start ========================
    SetVel_DtLeft_DtRight(70, 70);
    DT_Left.spinFor(reverse, 50, degrees, false);
    DT_Right.spinFor(reverse, 50, degrees, true);
    DriveTrainStop(brake);
    DT_Left.spinFor(reverse, 35, deg, false);
    DT_Right.spinFor(forward, 35, deg, true);
    SetVel_DtLeft_DtRight(40, 40);
    DT_Left.spin(reverse);
    DT_Right.spin(reverse);
    wait(0.2, sec);
    ArmUp();
    Up = false;
    UpandDown();
    while (DT_Left.velocity(pct) < -5 && DT_Right.velocity(pct) < -5)
    {
        /* code */
        wait(20, msec);
    }
    wait(300, msec);
    // shoot
    DriveTrainStop(hold);
    Shot();
    // // // =============== Start step 2 ===================
    SetVel_DtLeft_DtRight(50, 50);
    DT_Left.spinFor(forward, 130, degrees, false);
    DT_Right.spinFor(forward, 130, degrees, true);
    DriveTrainStop(brake);
    wait(0.2, sec);
    // // =============== turn ===================
    TurnRight_h_v_m(164, 30, 5);
    DriveTrainStop(brake);
    // =============== wailk in ===================
    SetVel_DtLeft_DtRight(30, 30);
    DT_Left.spinFor(forward, 100, degrees, false);
    DT_Right.spinFor(forward, 100, degrees, true);
    IntakeReload = true;
    wait(0.4, sec);
    // =============== wailk out ===================
    DT_Left.spinFor(reverse, 120, degrees, false);
    DT_Right.spinFor(reverse, 120, degrees, true);
    wait(0.2, sec);
    // // =============== turn ===================
    TurnRight_h_v_m(322, 30, 5);
    // =============== go shoot ===================
    SetVel_DtLeft_DtRight(60, 60);
    DT_Left.spin(reverse);
    DT_Right.spin(reverse);
    wait(0.2, sec);
    while (DT_Left.velocity(pct) < -5 && DT_Right.velocity(pct) < -5)
    {
        /* code */
        wait(20, msec);
    }
    wait(300, msec);
    // shoot
    DriveTrainStop(hold);
    Shot();
    // // // =============== Start step 3 ===================
    SetVel_DtLeft_DtRight(50, 50);
    DT_Left.spinFor(forward, 80, degrees, false);
    DT_Right.spinFor(forward, 80, degrees, true);
    DriveTrainStop(brake);
    wait(0.2, sec);
    SetVel_DtLeft_DtRight(10, 70);
    // DT_Left.spin(forward);
    DT_Left.stop();
    DT_Right.spin(forward);
    wait(0.35, sec);
    TurnLeft_h_v_m(204, 30, 5);
    // =============== wailk in ===================
    SetVel_DtLeft_DtRight(30, 30);
    DT_Left.spinFor(forward, 240, degrees, false);
    DT_Right.spinFor(forward, 240, degrees, true);
    IntakeReload = true;
    wait(0.4, sec);
    // =============== wailk out ===================
    DT_Left.spinFor(reverse, 230, degrees, false);
    DT_Right.spinFor(reverse, 230, degrees, true);
    wait(0.2, sec);
    // // =============== turn ===================
    TurnRight_h_v_m(325, 30, 5);
    // =============== go shoot ===================
    SetVel_DtLeft_DtRight(60, 60);
    DT_Left.spin(reverse);
    DT_Right.spin(reverse);
    wait(0.2, sec);
    while (DT_Left.velocity(pct) < -5 && DT_Right.velocity(pct) < -5)
    {
        /* code */
        wait(20, msec);
    }
    wait(300, msec);
    // shoot
    DriveTrainStop(hold);
    Shot();
    // =============== Start step 4 ===================
    SetVel_DtLeft_DtRight(100, 100);
    DT_Left.spinFor(forward, 80, degrees, false);
    DT_Right.spinFor(forward, 80, degrees, true);
    DT_Left.setStopping(brake);
    DT_Right.setStopping(brake);
    DT_Right.stop();
    DT_Left.spin(forward);
    wait(900, msec);
    DT_Left.stop();
    if (BrainInertial.heading() < 50)
    {
        TurnRight_h_v_m(65, 30, 5);
    }
    else if (BrainInertial.heading() > 70)
    {
        TurnLeft_h_v_m(65, 30, 5);
    }
    SetVel_DtLeft_DtRight(50, 50);
    DT_Left.spinFor(forward, 220, degrees, false);
    DT_Right.spinFor(forward, 220, degrees, true);
    IntakeReload = true;
    wait(1, sec);
    // =============== Go L3 ===================
    // Up = true;
    // UpandDown();
    TurnRight_h_v_m(120, 100, 10);
    L3();
    DriveTrainStop(brake);
    Up = false;
    UpandDown();
    SetVel_DtLeft_DtRight(100, 80);
    DT_Left.spin(forward);
    DT_Right.spin(forward);
    wait(0.8, sec);
    IntakeReload = true;
    wait(0.2, sec);
    DriveTrainStop(brake);
    wait(0.8, sec);
    TurnRight_h_v_m(110, 100, 10);
    L3();
    DriveTrainStop(brake);
    wait(0.5, sec);
    // PIDfind(0);
    // autoL3 = new task(findBeamBag);
    autoL3 = new task(findBeamBag2);
}

int CalibrateInertial()
{
    Pneumatic10.pumpOn();
    Pneumatic10.retract(cylinder1);
    Pneumatic10.retract(cylinder2);
    Pneumatic4.extend(cylinder2);
    Pneumatic4.extend(cylinder1);
    Intake1.setVelocity(100.0, percent);
    Intake1.spin(reverse);
    wait(1.5, sec);
    Intake1.setPosition(0, degrees);
    Intake1.setStopping(brake);
    Intake1.stop();
    Shot();
    reload = true;
    wait(1, sec);
    Flip = true;
    ArmUp();
    wait(200, msec);
    SetVel_DtLeft_DtRight(20.0, 20.0);
    DT_Left.spin(forward);
    DT_Right.spin(forward);
    wait(0.5, sec);
    // start calibration
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating...");
    // start calibration
    Pneumatic10.pumpOff();
    Touchled8.setColor(red);
    BrainInertial.calibrate();
    // wait.....
    while (BrainInertial.isCalibrating())
    {
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Please wait...   ");
        wait(20, msec);
    }
    // calibration complete
    Touchled8.setColor(green);
    wait(1000, msec);
    Pneumatic10.pumpOn();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibration Done!");
    // read heading
    Start();
    while (true)
    {
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Heading : %6.2f   ", BrainInertial.heading());
        int dd = DT_Left.velocity(percent);
        int ff = DT_Right.velocity(percent);
        printf("\n%d", dd);
        printf("\n%d", ff);
        if (start)
        {
            if (Brain.Timer.value() > 60)
            {
                Brain.programStop();
            }
        }
        task::sleep(10);
    }

    return 0;
}
// void findBeamBag1()
// {
//     while (true)
//     {
//         DT_Left.setStopping(coast);
//         DT_Right.setStopping(coast);
//         if (count_Bag <= 6.0)
//         {
//             AIVision5.takeSnapshot(COLORR);
//             RedBlue = true;
//             Touchled8.setColor(orange);
//         }
//         if (AIVision5.objectCount > 0)
//         {
//             SetVel_DtLeft_DtRight(20, 20);
//             DT_Left.spin(forward);
//             DT_Right.spin(forward);
//             wait(0.07, sec);

//             for (int i = 0; i < AIVision5.objectCount; i++)
//             {
//                 if (AIVision5.objects[i].centerY > maxY)
//                 {
//                     maxY = AIVision5.objects[i].centerY;
//                     closestIndex = i;
//                 }
//             }

//             Obj_Now = AIVision5.objects[closestIndex].centerY;
//             // centerX = AIVision5.objects[closestIndex].centerX;
//             if (Obj_Now > Obj_after)
//             {
//                 // เข้ามาใกล้ ?
//                 if (Obj_Now > 182)
//                 {
//                     if (count_Bag <= 6.0)
//                     {
//                         // start
//                         // grap
//                         IntakeReload = true;
//                         wait(0.1, sec);
//                         DriveTrainStop(brake);
//                         wait(0.3, sec);
//                         // load done ?
//                         DT_Right.spinFor(reverse, 110.0, degrees, false);
//                         while (!beambag)
//                         {
//                             /* code */
//                             wait(20, msec);
//                         }
//                         // wait(0.3, sec);
//                         // DT_Left.spinFor(reverse, 80.0, degrees, true);
//                     }
//                     // start วิ่ง
//                     if (count_Bag <= 6.0 && RedBlue)
//                     {
//                         L3();
//                         count_Bag += 1;
//                         // Color2.setLight(0, percent);
//                         maxY = 0;
//                         closestIndex = 0;
//                         Obj_Now = 0;
//                         Obj_after = 0;
//                         return;
//                     }
//                     else
//                     {
//                         Brain.programStop();
//                     }
//                 }
//             }
//             else
//             {
//                 Touchled8.setColor(black);
//                 SetVel_DtLeft_DtRight(40, 40);
//                 DT_Left.spin(forward);
//                 DT_Right.spin(forward);
//                 Obj_Now = 0;
//                 Obj_after = 0;
//             }
//         }
//         else
//         {
//             Touchled8.setColor(black);
//             DT_Left.spin(forward);
//             DT_Right.spin(forward);
//             Obj_after = 0.0;
//             Obj_Now = 0.0;
//         }
//         // wait(10, msec);
//     }
// }

// int findBeamBag()
// {
//     while (true)
//     {
//         DT_Left.setStopping(coast);
//         DT_Right.setStopping(coast);
//         if (count_Bag <= 6.0)
//         {
//             AIVision5.takeSnapshot(COLORR);
//             RedBlue = true;
//             Touchled8.setColor(red);
//         }
//         if (count_Bag >= 7.0)
//         {
//             AIVision5.takeSnapshot(COLORB);
//             RedBlue = false;
//             Touchled8.setColor(blue);
//         }
//         if (AIVision5.objectCount > 0)
//         {
//             SetVel_DtLeft_DtRight(70, 70);
//             DT_Left.setMaxTorque(100, pct);
//             DT_Right.setMaxTorque(80, pct);
//             DT_Left.spin(forward);
//             DT_Right.spin(forward);
//             wait(0.5, sec);

//             for (int i = 0; i < AIVision5.objectCount; i++)
//             {
//                 if (AIVision5.objects[i].centerY > maxY)
//                 {
//                     maxY = AIVision5.objects[i].centerY;
//                     closestIndex = i;
//                 }
//             }

//             Obj_Now = AIVision5.objects[closestIndex].centerY;
//             // centerX = AIVision5.objects[closestIndex].centerX;
//             if (Obj_Now > Obj_after)
//             {
//                 // เข้ามาใกล้ ?
//                 if (Obj_Now > 170 || DT_Left.velocity(percent) <= 50 || DT_Right.velocity(percent) <= 50)
//                 {
//                     if (count_Bag <= 6.0)
//                     {
//                         // start
//                         // grap
//                         IntakeReload = true;
//                         wait(0.1, sec);
//                         DriveTrainStop(brake);
//                         wait(0.3, sec);
//                         // load done ?
//                         DT_Right.spinFor(reverse, 110.0, degrees, false);
//                         while (!beambag)
//                         {
//                             /* code */
//                             wait(20, msec);
//                         }
//                         // wait(0.3, sec);
//                         // DT_Left.spinFor(reverse, 80.0, degrees, true);
//                     }
//                     // start วิ่ง
//                     if (count_Bag <= 6.0 && RedBlue)
//                     {
//                         L3();
//                         count_Bag += 1;
//                         // Color2.setLight(0, percent);
//                         maxY = 0;
//                         closestIndex = 0;
//                         Obj_Now = 0;
//                         Obj_after = 0;
//                     }
//                     else
//                     {
//                         Brain.programStop();
//                     }
//                 }
//             }
//             else
//             {
//                 Touchled8.setColor(black);
//                 SetVel_DtLeft_DtRight(70, 70);
//                 DT_Left.spin(forward);
//                 DT_Right.spin(forward);
//                 wait(10, msec);
//             }
//             // }
//         }
//         else
//         {
//             DT_Left.stop();
//             DT_Right.stop();
//         }
//     }
//     return 0;
// }

int findBeamBag2()
{
    int count_Bag = 0;
    // ---------------- funtion shoot ----------------
    while (count_Bag <= 3)
    {
        // Touchled8.setColor(yellow_orange);
        int speedAuto = 127;
        waitTimeforshoot = 1000;
        SetVel_DtLeft_DtRight(100, 100);
        // DT_Left.spin(reverse);
        // DT_Right.spin(forward);
        // wait(100, msec);
        DT_Left.spin(forward);
        DT_Right.spin(forward);
        Touchled8.setColor(blue);
        while (distance5.objectDistance(mm) > 245)
        {
            wait(20, msec);
        }
        // Intake
        // Pneumatic10.extend(cylinder1);
        IntakeReload = true;
        DriveTrainStop(brake);
        speedAuto = -30;
        int count = 0;
        Color2.setLight(100, percent);
        wait(50, msec);
        SetVel_DtLeft_DtRight(speedAuto, 127);
        // ---- Down L4 ---------
        Up = true;
        UpandDown();
        // -----Go shoot L3---------
        DT_Left.spin(reverse);
        DT_Right.spin(reverse);
        wait(0.2, seconds);
        if (speedAuto == -30)
        {
            SetVel_DtLeft_DtRight(100.0, 100.0);
            DT_Left.spin(reverse);
            DT_Right.spin(reverse);
        }
        while (Color2.hue() >= 90.0 || fabs(DT_Right.velocity(percent) >= 40.0))
        { // check color yellow check velocity drivetrainRight 10 percent
        }
        // ------- adjust angle ---------
        Pneumatic4.retract(cylinder1);
        // wait(0.2, seconds);
        DT_Left.spin(forward);
        DT_Right.spin(reverse);
        wait(0.2, seconds);
        DriveTrainStop(brake);
        // wait(0.1, seconds);
        // --------- shoot --------------
        Catapult7.spin(reverse);
        wait(0.2, seconds);
        // --------- Start Exit ---------
        reload = true;
        // Pneumatic4.extend(cylinder2);
        Pneumatic4.extend(cylinder1);
        wait(0.15, seconds);
        SetVel_DtLeft_DtRight(100.0, 100.0);
        DT_Left.spin(forward);
        DT_Right.spin(forward);
        while ((Color2.hue() <= 90.0))
        { // check color yellow check velocity drivetrainRight 10 percent
            if (count == 30)
            {
                SetVel_DtLeft_DtRight(100.0, 100.0);
                DT_Left.spin(reverse);
                wait(100, msec);
                count = 0;
            }
            DT_Left.spin(forward);
            DT_Right.spin(forward);
            wait(20, msec);
            count += 1;
        }
        DT_Left.stop();
        DT_Right.stop();
        SetVel_DtLeft_DtRight(100, 100);
        DT_Left.spin(forward);
        DT_Right.spin(forward);
        wait(0.2, seconds);
        SetVel_DtLeft_DtRight(70, 70);
        DT_Left.spin(reverse);
        DT_Right.spin(forward);
        wait(0.2, seconds);
        DT_Left.stop();
        DT_Right.stop();
        wait(0.1, seconds);
        Color2.setLight(0, percent);
        count_Bag += 1;
    }
    return 0;
}

// void Codeshoot()
// {
//   passed = true;
//   wasd = false;
//   fire = false;
//   if (StopAutoCheck())
//     return;
//   SetVel_DtLeft_DtRight(92.0, 100.0);
//   DT_Left.spin(reverse);
//   DT_Right.spin(reverse);
//   Catapult7.stop();
//   for (int t = 0; t < 120 && AutoRunEnable; t++)
//   {
//     task::sleep(10);
//   }
//   DT_Left.stop(coast);
//   // wait(50, msec);
//   DT_Right.stop(coast);
//   Touchled8.setColor(green);
//   Catapult7.spin(reverse);
//   wait(0.2, sec);
//   if (StopAutoCheck())
//     return;
//   Catapult7.spin(forward);
//   Touchled8.setColor(red);
//   SetVel_DtLeft_DtRight(30.0, 100.0);
//   DT_Left.spin(forward);
//   DT_Right.spin(forward);
//   wait(0.3, sec);
//   if (StopAutoCheck())
//     return;
//   fire = true;
//   passed = false;
// }

// void AutoRun()
// {
//   double Torque = 5;
//   wasd = false;
//   AutoReload = false;
//   Pneumatic10.extend(cylinder2);
//   while (AutoRunEnable)
//   {
//     DT_Left.setStopping(brake);
//     DT_Right.setStopping(brake);
//     SetVel_DtLeft_DtRight(80.0, 100.0);
//     DT_Left.spin(forward);
//     DT_Right.spin(forward);
//     wait(0.2, seconds);
//     while (AutoRunEnable)
//     {
//       if (fabs(DT_Right.velocity(percent)) <= 1.0 && fabs(DT_Left.velocity(percent)) <= 1.0)
//       {
//         break;
//       }

//       if ((4 < distance5.objectDistance(inches)) and (distance5.objectDistance(inches)) < 12)
//       {
//         SetVel_DtLeft_DtRight(40, 40);
//         DT_Left.spin(forward);
//         DT_Right.spin(forward);
//         DT_Left.setMaxTorque(Torque, pct);
//         DT_Right.setMaxTorque(Torque, pct);
//       }
//       DT_Left.spin(forward);
//       DT_Right.spin(forward);
//       task::sleep(10);
//     }
//     if (!AutoRunEnable)
//     {
//       break;
//     }
//     Pneumatic10.retract(cylinder2);
//     FDown_pressed();
//     Pneumatic10.extend(cylinder2);
//     DriveTrainStop_SetStop(brake);
//     DT_Left.setMaxTorque(100, pct);
//     DT_Right.setMaxTorque(100, pct);
//     SetVel_DtLeft_DtRight(95.0, 100.0);
//     DT_Left.stop();
//     DT_Right.spin(reverse);
//     wait(100, msec);
//     DT_Right.stop();
//     // wait(2000000, sec);
//     Codeshoot();
//     task::sleep(20);
//   }
//   Pneumatic10.retract(cylinder2);
//   DT_Left.setStopping(brake);
//   DT_Right.setStopping(brake);
//   fire = true;
//   passed = false;
//   AutoReload = true;
//   reload = true;
//   wasd = true;
// }

// int findBeamBag()
// {
//     while (true)
//     {
//         DT_Left.setStopping(coast);
//         DT_Right.setStopping(coast);

//         // Detect target color
//         AIVision5.takeSnapshot(COLORR);
//         RedBlue = true;
//         Touchled8.setColor(count_Bag == 0 ? orange : red);

//         if (AIVision5.objectCount == 0)
//         {
//             DT_Left.spin(forward);
//             DT_Right.spin(forward);
//             wait(20, msec);
//             continue;
//         }

//         // Reset closest target selection every snapshot
//         maxY = -1;
//         closestIndex = 0;

//         // Find the closest bean bag (largest centerY)
//         for (int i = 0; i < AIVision5.objectCount; i++)
//         {
//             if (AIVision5.objects[i].centerY > maxY)
//             {
//                 maxY = AIVision5.objects[i].centerY;
//                 closestIndex = i;
//             }
//         }

//         Obj_Now = AIVision5.objects[closestIndex].centerY;

//         // First two bags use a slower approach for better alignment
//         int speed = (count_Bag < 2) ? 20 : 100;

//         // ===== TUNE THIS VALUE =====
//         // Adjust targetY based on field testing.
//         // Increase if the robot grabs too early.
//         // Decrease if it grabs too late.
//         int targetY = (count_Bag < 2) ? 185 : 170;

//         DT_Left.setMaxTorque(70, pct);
//         DT_Right.setMaxTorque(70, pct);
//         SetVel_DtLeft_DtRight(speed, speed);

//         DT_Left.spin(forward);
//         DT_Right.spin(forward);

//         if (Obj_Now > Obj_after)
//         {
//             // Stop when the target is close enough or the robot stalls on contact
//             if (Obj_Now > targetY ||
//                 (count_Bag > 0 &&
//                  (fabs(DT_Left.velocity(percent)) <= 50 ||
//                   fabs(DT_Right.velocity(percent)) <= 50)))
//             {
//                 IntakeReload = true;
//                 wait(200, msec);

//                 if (count_Bag == 1)
//                     DriveTrainStop(coast);
//                 else
//                     DriveTrainStop(brake);

//                 wait(1200, msec);

//                 DT_Right.spinFor(reverse, 110, degrees, false);

//                 while (!beambag)
//                     wait(20, msec);

//                 if (RedBlue)
//                 {
//                     L3();
//                     wait(200, msec);
//                     count_Bag++;
//                 }

//                 // Reset tracking for the next target
//                 Obj_Now = 0;
//                 Obj_after = 0;
//                 maxY = -1;
//                 closestIndex = 0;
//             }
//         }
//         else
//         {
//             Touchled8.setColor(black);
//             SetVel_DtLeft_DtRight(70, 70);
//             DT_Left.spin(forward);
//             DT_Right.spin(forward);
//             wait(10, msec);
//         }

//         // Store the previous centerY for the next loop
//         Obj_after = Obj_Now;
//     }

//     return 0;
// }

void Shot()
{
    // if (beambag && reloadcomplete)
    // {
    Catapult7.spin(reverse);
    wait(0.2, seconds);
    reload = true;
    // beambag = false;
    // }
}

int Unnecessary_step()
{
    while (true)
    {
        if (IntakeReload)
        {
            Intake();
            beambag = true;
            IntakeReload = false;
        }
        if (reload)
        {
            Catareload();
            reloadcomplete = true;
        }
        if (InTakeUp)
        {
            IntakeUp();
        }

        task::sleep(10);
    }
    return 0;
}

int main()
{
    // Initializing Robot Configuration. DO NOT REMOVE!

    // register event handlers
    Touchled8.pressed(YL4);

    wait(15, msec);
    vex::task ws1(CalibrateInertial);
    vex::task ws2(Unnecessary_step);
}