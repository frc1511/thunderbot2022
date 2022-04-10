#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include "Drive.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"
#include "BlinkyBlinky.h"
#include "ControllerState.h"
#include "ShotMath.h"
#include <frc/Joystick.h>

class Controls : public Mechanism {
public:
    Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang, Limelight* limelight, BlinkyBlinky* blinkyBlinky);

    ~Controls();

    void resetToMode(MatchMode mode);
    void process() override;
    void sendFeedback() override;
    bool getShouldPersistConfig();
    // :D
    void autoForTrevor();

    void controllerInDisable();

    void chooseAutoMode(int autoMode);

private:

    void doDrive();
    void doAux();
    void doSwitchPanel();
    Drive* drive;
    GamEpiece* gamEpiece;
    Hang* hang;
    Limelight* limelight;
    BlinkyBlinky* blinkyBlinky;
    ShotMath shotmath{};
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
    bool nearFarShot = false;
    bool highOrLow = false; //true = high hub shot, false = low hub shot
    bool nearOrFar = false; //for launch pad true = near launch pad shot, false = far launch pad shot
    bool peterCentric = false;

    bool recordController = false;
    bool clearController = false;

//Normal Aux Variables
    Shooter::ShooterMode lastPressedMode = Shooter::ODOMETRY;
//Manual Aux Variables
    int lastDPadValue = -1;

    int theAutoMode = 0;
    // Something here...
    ControllerState driveController{0};
    ControllerState auxController{1};

};