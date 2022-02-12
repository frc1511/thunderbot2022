#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"
#include <frc/Joystick.h>

class Controls : public Mechanism {
public:
    Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang);

    ~Controls();

    void process() override;


private:

    void doDrive();
    void doAux();
    void doSwitchPanel();
    Drive* drive;
    GamEpiece* gamEpiece;
    Hang* hang;
    bool cameraWasToggled = false;
    bool offsetsWereConfigured = false;

    frc::Joystick controllerDrive{0};
    frc::Joystick controllerAux{1};
    frc::Joystick switchPanel{2};

//switchPanel Variables
    bool hangActive = false;
    bool hangManual = false;
    bool gamePieceManual = false;

//Normal Aux Variables
    Shooter::ShooterMode lastPressedMode = Shooter::TARMAC_LINE;
//Manual Aux Variables
    int lastDPadValue = -1;
    // Something here...
};