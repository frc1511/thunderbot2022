#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"
#include "ControllerState.h"
#include <frc/Joystick.h>

class Controls : public Mechanism {
public:
    Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang);

    ~Controls();

    void resetToMode(MatchMode mode);
    void process() override;
    void sendFeedback() override;
    bool getShouldPersistConfig();
    // :D
    void autoForTrevor();

    void controllerInDisable();

private:

    void doDrive();
    void doAux();
    void doSwitchPanel();
    Drive* drive;
    GamEpiece* gamEpiece;
    Hang* hang;
    bool cameraWasToggled = false;
    bool offsetsWereConfigured = false;

    frc::Joystick switchPanel{2};

    // false is limelight, true is camera
    bool whichCamera = true;

    bool shoot = false;

//switchPanel Variables
    bool hangActive = false;
    bool hangManual = false;
    bool gamePieceManual = false;
    bool robotCentric = false;
    bool highLowShot = false;
    bool highOrLow = false; //true = high hub shot, false = low hub shot

//Normal Aux Variables
    Shooter::ShooterMode lastPressedMode = Shooter::TARMAC_LINE;
//Manual Aux Variables
    int lastDPadValue = -1;
    // Something here...
    ControllerState driveController{0};
    ControllerState auxController{1};

};