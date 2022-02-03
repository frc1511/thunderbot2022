#pragma once

#include <frc/Joystick.h>

class ControllerState{
    public:

    ControllerState(int contollerID);
    ~ControllerState();
    


    bool getRawButton(int buttonID);
    double getRawAxis(int axisID);

    void setRawButton(int buttonID);
    void setRawAxis(int axisID);

    void updateFromJoystic();

    private:
    bool normalOrRelay = true; // normal controller controls is true, relay mode is false

    frc::Joystick myController;


};