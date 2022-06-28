#pragma once
#include "Feedback.h"
#include <frc/Joystick.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <frc/Timer.h>

//#define XBOX_CONTROLLER

class ControllerState{
    public:

    ControllerState(int contollerID);
    ~ControllerState();

    void process();
    void reset();
    void sendFeedback();

    // .GetRawBUtton but with copntrollerState
    bool getRawButton(int buttonID);
    // .GetRawAxis but with controller state. .GetPOV() is .getRawAxis(6)
    double getRawAxis(int axisID);
    // only triggers when the button gets pressed down
    bool getRawButtonPressed(int buttonID);
    // only triggers when the button gets released
    bool getRawButtonReleased(int buttonID);

    
    // prints the contents of autoButtons and autoAxes text files, uses those to store recorded inputs
    void testStuff();
    // clear the current selected mode
    void clearAuto();

    // replays the current inputs that it has stored
    void replayAuto();
    // starts/stops recording the current mode. its a toggle so only call once
    void record();

    void chooseAutoMode(int whichAutoMode);

    private:
    // records the current button 
    void recordButton(int whichButton, double time);
    // records the current axis and the pov
    void recordAxis(int whichButton, double time, double position);
    // the text files that it will use to do stuff
    const char* autoButtonsFile = "";
    const char* autoAxesFile = "";
    // normal controller controls is true, relay mode is false
    bool normalOrRelay = true; 

    frc::Joystick myController;
    std::vector<const char*> driverAutoFiles = {"/home/lvuser/firstDriverAutoButtons.txt", "/home/lvuser/firstDriverAutoAxes.txt", "/home/lvuser/secondDriverAutoButtons.txt", "/home/lvuser/secondDriverAutoAxes.txt", "/home/lvuser/thirdDriverAutoButtons.txt", "/home/lvuser/thirdDriverAutoAxes.txt", "/home/lvuser/fourthDriverAutoButtons.txt", "/home/lvuser/fourthDriverAutoAxes.txt", "/home/lvuser/fifthDriverAutoButtons.txt", "/home/lvuser/fifthDriverAutoAxes.txt", "/home/lvuser/sixthDriverAutoButtons.txt", "/home/lvuser/sixthDriverAutoAxes.txt", "/home/lvuser/seventhDriverAutoButtons.txt", "/home/lvuser/seventhDriverAutoAxes.txt", "/home/lvuser/eighthDriverAutoButtons.txt", "/home/lvuser/eighthDriverAutoAxes.txt", "/home/lvuser/ninthDriverAutoButtons.txt", "/home/lvuser/ninthDriverAutoAxes.txt", "/home/lvuser/tenthDriverAutoButtons.txt", "/home/lvuser/tenthDriverAutoAxes.txt"}; 
    std::vector<const char*> auxAutoFiles = {"/home/lvuser/firstAuxAutoButtons.txt", "/home/lvuser/firstAuxAutoAxes.txt", "/home/lvuser/secondAuxAutoButtons.txt", "/home/lvuser/secondAuxAutoAxes.txt", "/home/lvuser/thirdAuxAutoButtons.txt", "/home/lvuser/thirdAuxAutoAxes.txt", "/home/lvuser/fourthAuxAutoButtons.txt", "/home/lvuser/fourthAuxAutoAxes.txt", "/home/lvuser/fifthAuxAutoButtons.txt", "/home/lvuser/fifthAuxAutoAxes.txt", "/home/lvuser/sixthAuxAutoButtons.txt", "/home/lvuser/sixthAuxAutoAxes.txt", "/home/lvuser/seventhAuxAutoButtons.txt", "/home/lvuser/seventhAuxAutoAxes.txt", "/home/lvuser/eighthAuxAutoButtons.txt", "/home/lvuser/eighthAuxAutoAxes.txt", "/home/lvuser/ninthAuxAutoButtons.txt", "/home/lvuser/ninthAuxAutoAxes.txt", "/home/lvuser/tenthAuxAutoButtons.txt", "/home/lvuser/tenthAuxAutoAxes.txt"};

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

    int amountOfButtons = 14;
    bool recordOrNot;

    
    int whichController;
    int whichMode = 0;
    frc::Timer autoTimer;



};