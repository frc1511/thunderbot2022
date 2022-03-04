#include "Autonomous.h"

#define LEFT_START_POSITION   (frc::Pose2d(1.838_m, 6.142_m, 226.511_deg - 90_deg))
#define CENTER_START_POSITION (frc::Pose2d(5.136_m, 5.959_m, 313.468_deg - 90_deg))
#define RIGHT_START_POSITION  (frc::Pose2d(6.015_m, 8.124_m, 1.498_deg - 90_deg))

Autonomous::Autonomous(Drive* drive, GamEpiece* gamEpiece, Controls* controls)
  : drive(drive), gamEpiece(gamEpiece), controls(controls) {
    
}

Autonomous::~Autonomous() {

}

void Autonomous::resetToMode(MatchMode mode) {
    currentMode = AUTO_FOR_TREVOR_TWO;

    if (mode == MODE_AUTO) {
        currentMode = (AutoMode)Feedback::getDouble("Auto", "Mode", 0);
        
        switch (currentMode) {
            case DO_NOTHING:
            case UBER:
                startPosition = UNKNOWN;
                break;
            case ONE_BALL:
                startPosition = UNKNOWN;
                break;
            case LEFT_TWO_BALL:
                startPosition = LEFT;
                break;
            case CENTER_TWO_BALL:
            case CENTER_THREE_BALL:
                startPosition = CENTER;
                break;
            case RIGHT_TWO_BALL:
            case RIGHT_SHORT_THREE_BALL:
            case RIGHT_FAR_THREE_BALL:
            case RIGHT_FOUR_BALL:
                startPosition = RIGHT;
                break;
            case AUTO_FOR_TREVOR_ZERO:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(0);
                break;
            case AUTO_FOR_TREVOR_ONE:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(1);
                break;
            case AUTO_FOR_TREVOR_TWO:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(2);
                break;
            case AUTO_FOR_TREVOR_THREE:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(3);
                break;
            case AUTO_FOR_TREVOR_FOUR:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(4);
                break;
            case AUTO_FOR_TREVOR_FIVE:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(5);
                break;
            case AUTO_FOR_TREVOR_SIX:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(6);
                break;
            case AUTO_FOR_TREVOR_SEVEN:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(7);
                break;
            case AUTO_FOR_TREVOR_EIGHT:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(8);
                break;
            case AUTO_FOR_TREVOR_NINE:
                startPosition = UNKNOWN;
                controls->chooseAutoMode(9);
                break;
        }
        
        drive->zeroRotation();

        switch (startPosition) {
            case UNKNOWN:
                drive->resetOdometry({});
                break;
            case LEFT:
                drive->resetOdometry(LEFT_START_POSITION);
                break;
            case CENTER:
                drive->resetOdometry(CENTER_START_POSITION);
                break;
            case RIGHT:
                drive->resetOdometry(RIGHT_START_POSITION);
                break;
        }

        timer.Reset();
        timer.Start();
        step = 0;
        shootStep = 0;
        shootingIsDone = false;
        
        switch (currentMode) {
            case DO_NOTHING:
            case UBER:
            case ONE_BALL:
            case LEFT_TWO_BALL:
            case CENTER_TWO_BALL:
            case CENTER_THREE_BALL:
            case RIGHT_TWO_BALL:
            case RIGHT_SHORT_THREE_BALL:
            case RIGHT_FAR_THREE_BALL:
            case RIGHT_FOUR_BALL:
                break;
            case AUTO_FOR_TREVOR_ZERO:
                controls->chooseAutoMode(0);
                break;
            case AUTO_FOR_TREVOR_ONE:
                controls->chooseAutoMode(1);
                break;
            case AUTO_FOR_TREVOR_TWO:
                controls->chooseAutoMode(2);
                break;
            case AUTO_FOR_TREVOR_THREE:
                controls->chooseAutoMode(3);
                break;
            case AUTO_FOR_TREVOR_FOUR:
                controls->chooseAutoMode(4);
                break;
            case AUTO_FOR_TREVOR_FIVE:
                controls->chooseAutoMode(5);
                break;
            case AUTO_FOR_TREVOR_SIX:
                controls->chooseAutoMode(6);
                break;
            case AUTO_FOR_TREVOR_SEVEN:
                controls->chooseAutoMode(7);
                break;
            case AUTO_FOR_TREVOR_EIGHT:
                controls->chooseAutoMode(8);
                break;
            case AUTO_FOR_TREVOR_NINE:
                controls->chooseAutoMode(9);
                break;
        }
    }
}

static void handleDashboardString(Autonomous::AutoMode mode, const char* description, char* buffer) {
    // Append mode number to the end of the buffer.
    sprintf(&buffer[strlen(buffer)], ",%d", mode);

    char mode_str[32];

    // Convert the mode into a string.
    sprintf(mode_str, "%d", mode);
    
    Feedback::sendString("thunderdashboard_auto", mode_str, description);
}

void Autonomous::sendFeedback() {
    Feedback::sendDouble("autonomous", "current step", step);
    Feedback::sendDouble("autonomous", "shooter step", shootStep);
    Feedback::sendBoolean("autonomous", "align and shoot complete", shootingIsDone);
    char buffer[256] = "";

    handleDashboardString(DO_NOTHING, "Do nothing: Doing nothing or something; we don't know", buffer);

    handleDashboardString(UBER, "Uber: Drive backwards out of the tarmac (taxi)", buffer);

    handleDashboardString(ONE_BALL, "One ball: (Positioned facing the hub) Score ball in robot and drive backwards", buffer);

    handleDashboardString(LEFT_TWO_BALL, "Left Two Ball: (Positioned left)", buffer);
    handleDashboardString(CENTER_TWO_BALL, "Center Two Ball: (Positioned center)", buffer);
    handleDashboardString(RIGHT_TWO_BALL, "Rw 3221      ight Two Ball: (Positioned right)", buffer);
    //hi ishan
    handleDashboardString(CENTER_THREE_BALL, "Center three ball: (Positioned center)", buffer);
    handleDashboardString(RIGHT_SHORT_THREE_BALL, "Right short three ball: (Positioned right)", buffer);
    handleDashboardString(RIGHT_FAR_THREE_BALL, "Right far three ball: (Positioned right)", buffer);
    handleDashboardString(RIGHT_FOUR_BALL, "Right four ball: (Positioned right)", buffer);

    handleDashboardString(AUTO_FOR_TREVOR_ZERO, "First Auto For Trevor", buffer);
    handleDashboardString(AUTO_FOR_TREVOR_ONE, "Second Auto For Trevor", buffer);
    handleDashboardString(AUTO_FOR_TREVOR_TWO, "Third Auto For Trevor", buffer);
    //handleDashboardString(AUTO_FOR_TREVOR_THREE, "Fourth Auto For Trevor", buffer);
    //handleDashboardString(AUTO_FOR_TREVOR_FOUR, "Fifth Auto For Trevor", buffer);
    //handleDashboardString(AUTO_FOR_TREVOR_FIVE, "Sixth Auto For Trevor", buffer);
    //handleDashboardString(AUTO_FOR_TREVOR_SIX, "Seventh Auto For Trevor", buffer);
    //handleDashboardString(AUTO_FOR_TREVOR_SEVEN, "Eighth Auto For Trevor", buffer);
    //handleDashboardString(AUTO_FOR_TREVOR_EIGHT, "Ninth Auto For Trevor", buffer);
    //handleDashboardString(AUTO_FOR_TREVOR_NINE, "Tenth Auto For Trevor", buffer);


    Feedback::sendString("thunderdashboard", "auto_list", buffer);
}

void Autonomous::process() {
    if (!gamEpiece)
        return;

    if (timer.Get().value() <= Feedback::getDouble("thunderdashboard", "auto_start_delay", 0)) {
        return;
    }

    switch (currentMode) {
        case DO_NOTHING:
            doNothing();
            break;
        case UBER:
            uber();
            break;
        case ONE_BALL:
            oneBall();
            break;
        case LEFT_TWO_BALL:
            leftTwoBall();
            break;
        case CENTER_TWO_BALL:
            centerTwoBall();
            break;
        case RIGHT_TWO_BALL:
            rightTwoBall();
            break;
        case CENTER_THREE_BALL:
            centerThreeBall();
            break;
        case RIGHT_SHORT_THREE_BALL:
            rightShortThreeBall();
            break;
        case RIGHT_FAR_THREE_BALL:
            rightFarThreeBall();
            break;
        case RIGHT_FOUR_BALL:
            rightFourBall();
            break;
        case AUTO_FOR_TREVOR_ZERO:
        case AUTO_FOR_TREVOR_ONE:
        case AUTO_FOR_TREVOR_TWO:
        case AUTO_FOR_TREVOR_THREE:
        case AUTO_FOR_TREVOR_FOUR:
        case AUTO_FOR_TREVOR_FIVE:
        case AUTO_FOR_TREVOR_SIX:
        case AUTO_FOR_TREVOR_SEVEN:
        case AUTO_FOR_TREVOR_EIGHT:
        case AUTO_FOR_TREVOR_NINE:
            autoForTrevor();
            break;
    }
}

void Autonomous::doNothing() {
    // If it does nothing is it doing something or nothing? - trevor(2020)
        //it does something because it is doing nothing - ishan(2022)
        //I disagree - peter(2022)
        //I agree with peter -L Wrench
    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups
}

void Autonomous::uber() {
    if (step == 0) {
        drive->cmdDriveTranslate(0_m, 1_m, 0_deg);
        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
}

void Autonomous::oneBall() {
    if(step == 0){
        drive->cmdDriveTranslate(0_ft, -8_in, 0_deg);
        step++;
    }
    else if (step == 2) {
        alignAndShoot(Shooter::HIGH_HUB_SHOT);
        if(shootingIsDone){
            step++;
        }
    }
    else if ((step == 1 || step == 3 || step == 5) && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 4) {
        drive->cmdDriveTranslate(0_ft, -70_in, 0_deg, {});
        step++;
    }
}

void Autonomous::leftTwoBall() {
}

void Autonomous::centerTwoBall() {
}

void Autonomous::rightTwoBall() {
    if(step == 0) {
        drive->cmdDriveTranslate(0_in, -37.5_in + 17_in + 6_in, -90_deg);
        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
#ifndef HOMER
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        gamEpiece->setShooterWarmUpEnabled(Shooter::TARMAC_LINE, true);
#endif
        step++;
    }
    else if (step == 2) {
        drive->cmdDriveTranslate(42_in - 7_in - 3_in, 0_in, -90_deg);
        step++;
    }
    else if (step == 3 && gamEpiece->ballAtStageOne()) {
        step++;
    }
    else if (step == 4 && drive->cmdIsFinished()) {
#ifndef HOMER
        gamEpiece->setIntakeDirection(GamEpiece::NOTTAKE);
#endif
        step++;
    }
    else if (step == 5) {
        drive->cmdDriveTranslate(6_in, 0_in, 90_deg);
        step++;
    }
    else if (step == 6 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 7 || step == 9) {
        alignAndShoot(Shooter::TARMAC_LINE);
        if(shootingIsDone){
            step++;
        }
    }
    else if(step == 8) {
        drive->cmdDriveTranslate(0_in,0_in,95_deg);
        step++;
    }
    else if (step == 10) {
#ifndef HOMER
        gamEpiece->setShooterWarmUpEnabled(Shooter::TARMAC_LINE, false);
#endif
    }
}

void Autonomous::centerThreeBall() {
}

void Autonomous::rightShortThreeBall() {
    //hi peter p. lilley iii
    if(step == 0) {
        rightTwoBall();
    }
    else if(step == 11) {
        drive->cmdDriveTranslate(-38_in, -95_in, 2.76_rad);
        step++;
    }
    else if(step == 12 && drive->cmdIsFinished()) {
        drive->cmdDriveTranslate(-2_ft, 2_ft, 0.62_rad);
        step++;
    }
    else if(step == 13) {
        alignAndShoot(Shooter::TARMAC_LINE);
        if(shootingIsDone){
            step++;
        }
    }
}

void Autonomous::rightFarThreeBall() {
    if (step == 0) {
        rightTwoBall();
    }
}

void Autonomous::rightFourBall() {
    //hi nadia
    if(step == 9) {
        rightShortThreeBall();
        step++;
    }
    else if(step == 10) {
        drive->cmdDriveTranslate(10_ft, -3_ft, 100_deg, {});
        step++;
    }
    else if(step == 11 && drive->cmdIsFinished()){
        step++;
    }
    else if(step == 12) {
        drive->cmdDriveTranslate(10_ft, 10_ft, -100_deg, {});
        step++;
    }
    else if(step == 13 && drive->cmdIsFinished()){
        step++;
    }
}

void Autonomous::autoForTrevor(){
    controls->process();
    if(step == 0){
        controls->autoForTrevor();
        step++;
    }
}

void Autonomous::alignAndShoot(Shooter::ShooterMode shooterMode) {
    if (shootStep == 0) {
        // Start a command to align with high hub, and check if found by limelight.
        shootingIsDone = false;
        drive->cmdAlignToHighHub();
        shootStep++;
    }
    else if (shootStep == 1 && drive->cmdIsFinished()) {
        shootStep++;
#ifdef HOMER
        shootStep = 0;
        shootingIsDone = true;
#endif
    }
    else if (shootStep == 2) {
        gamEpiece->shootABall(shooterMode);
        shootStep++;
    }
    else if (shootStep == 3 && !gamEpiece->isShotInProgress()) {
        shootStep = 0;
        shootingIsDone = true;
    }
}

//                                       --- CAUTION ---
// What lies below is ishan's weird code that he did before we planned anything. Proceed with immense caution!
    //trevor
    //peter