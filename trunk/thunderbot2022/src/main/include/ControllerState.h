#pragma once

#include <frc/Joystick.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <frc/Timer.h>

class ControllerState{
    public:

    ControllerState(int contollerID);
    ~ControllerState();

    void process();
    void reset();


    bool getRawButton(int buttonID);
    double getRawAxis(int axisID);

    bool getRawButtonPressed(int buttonID);
    bool getRawButtonReleased(int buttonID);


    void setRawButton(int buttonID);
    void setRawAxis(int axisID);

    void updateFromJoystic();

    void recordButton(int whichButton, double time);
    void recordAxis(int whichButton, double time, double position);
    
    void testStuff();
    void clearAuto();

    void replayAuto();
    void record();

    private:
    
    bool normalOrRelay = true; // normal controller controls is true, relay mode is false

    frc::Joystick myController;

    std::vector<int> buttonsInt;
    std::vector<double> buttonsTime;
    std::vector<int> axesInt;
    std::vector<double> axesTime;
    std::vector<double> axesPos;
    std::vector<bool> buttonsValues = {false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    std::vector<double> axesValues = {0, 0, 0, 0, 0, 0, 0};
    std::vector<bool> buttons = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,};
    std::vector<double> axes = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int nextButtonsButton;
    double nextButtonsTime;
    int currentButtonThing;
    bool buttonReady;
    int nextAxesAxis;
    double nextAxesTime;
    double nextAxisPosition;
    int currentAxisThing;
    bool axisReady;
    bool buttonsDone;
    bool axisDone;

    bool timerStartedYet;

    bool recordOrNot;

    

    frc::Timer autoTimer;



};