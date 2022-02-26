#include "Autonomous.h"

#define LEFT_START_POSITION (frc::Pose2d(0_ft, 0_ft, 0_deg))
#define CENTER_START_POSITION (frc::Pose2d(0_ft, 0_ft, 0_deg))
#define RIGHT_START_POSITION (frc::Pose2d(0_ft, 0_ft, 0_deg))

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
                startPosition = UNKNOWN;
                break;
            case UBER:
                startPosition = UNKNOWN;
                break;
            case LEFT_ONE_BALL:
            case LEFT_TWO_BALL:
                startPosition = LEFT;
                break;
            case CENTER_ONE_BALL:
            case CENTER_TWO_BALL:
            case CENTER_THREE_BALL:
                startPosition = CENTER;
                break;
            case RIGHT_ONE_BALL:
            case RIGHT_TWO_BALL:
            case RIGHT_SHORT_THREE_BALL:
            case RIGHT_FAR_THREE_BALL:
            case RIGHT_FOUR_BALL:
                startPosition = RIGHT;
                break;
            case AUTO_FOR_TREVOR_ZERO:
            case AUTO_FOR_TREVOR_ONE:
            case AUTO_FOR_TREVOR_TWO:
                startPosition = UNKNOWN;
                break;
        }

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
    char buffer[256] = "";

    handleDashboardString(DO_NOTHING, "Doing nothing", buffer);

    handleDashboardString(UBER, "Driving out of the tarmac (taxi)", buffer);

    handleDashboardString(LEFT_ONE_BALL,   "(Positioned left) Score ball in robot", buffer);
    handleDashboardString(CENTER_ONE_BALL, "(Positioned center) Score ball in robot", buffer);
    handleDashboardString(RIGHT_ONE_BALL,  "(Positioned right) Score ball in robot", buffer);

    handleDashboardString(LEFT_TWO_BALL,   "(Positioned left) Score ball in robot and 1", buffer);
    handleDashboardString(CENTER_TWO_BALL, "(Positioned center) Score ball in robot, and 2", buffer);
    handleDashboardString(RIGHT_TWO_BALL,  "(Positioned right) Score ball in robot, and 3", buffer);
    //hi ishan
    handleDashboardString(CENTER_THREE_BALL,      "(Positioned center) Score ball 2, 4, and ball in robot", buffer);
    handleDashboardString(RIGHT_SHORT_THREE_BALL, "(Positioned right) Score ball in robot, 3, and 2,", buffer);
    handleDashboardString(RIGHT_FAR_THREE_BALL,   "(Positioned right) Score ball in robot, 3, and 4,", buffer);
    handleDashboardString(RIGHT_FOUR_BALL,        "(Positioned right) Score ball in robot, 3, 2, and 4", buffer);
    handleDashboardString(AUTO_FOR_TREVOR_ZERO, "first auto for trevor", buffer);
    handleDashboardString(AUTO_FOR_TREVOR_ONE, "second auto for trevor", buffer);
    handleDashboardString(AUTO_FOR_TREVOR_TWO, "third auto for trevor", buffer);

    Feedback::sendString("thunderdashboard", "auto_list", buffer);
}

void Autonomous::process() {
    // if (!gamEpiece)
    //     return;

    if (timer.Get().value() <= Feedback::getEditableDouble("thunderdashboard", "auto_start_delay", 0))
        {
            return;
        }

    switch (currentMode) {
        case DO_NOTHING:
            doNothing();
            break;
        case UBER:
            uber();
            break;
        case LEFT_ONE_BALL:
            leftOneBall();
            break;
        case CENTER_ONE_BALL:
            centerOneBall();
            break;
        case RIGHT_ONE_BALL:
            rightOneBall();
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
            controls->chooseAutoMode(0);
            autoForTrevor();
            break;
        case AUTO_FOR_TREVOR_ONE:
            controls->chooseAutoMode(0);
            autoForTrevor();
            break;
        case AUTO_FOR_TREVOR_TWO:
            controls->chooseAutoMode(0);
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
}

void Autonomous::uber() {
    if (step == 0) {
        std::cout << "hi\n";
        drive->cmdDrive(0_m, 2_m, 0_deg, PetersTrajectoryConfig());
        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 2) {
        drive->cmdDrive(-1_m, 0_m, 90_deg, PetersTrajectoryConfig());
        step++;
    }
    else if (step == 3 && drive->cmdIsFinished()) {
        step++;
    }
}

void Autonomous::leftOneBall() {
    if (step == 0) {
        // TODO: Initialize odometry to starting position.
        
        // Drive back to until outside of the tarmac.
        /** TODO: Change these values. */
        drive->cmdDrive(0_ft, 10_ft, 180_deg, {});

        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 2) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 1)) {
            step++;
        }
    }
    else if(step == 3 && drive->cmdIsFinished()) {
        step++;
    }
}

void Autonomous::centerOneBall() {
     if (step == 0) {
        // TODO: Initialize odometry to starting position.
        
        // Drive back to until outside of the tarmac.
        /** TODO: Change these values. */
        drive->cmdDrive(0_ft, 10_ft, 180_deg, {});

        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 2) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 1)) {
            step++;
        }
    }
    else if(step == 3 && drive->cmdIsFinished()) {
        step++;
    }
}

void Autonomous::rightOneBall() {
     if (step == 0) {
        
        // Drive back to until outside of the tarmac.
        /** TODO: Change these values. */
        drive->cmdDrive(0_ft, 10_ft, 180_deg, {});

        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 2) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 1)) {
            step++;
        }
    }
    else if(step == 3 && drive->cmdIsFinished()) {
        step++;
    }
}

void Autonomous::leftTwoBall() {
    if(step == 0) {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 1) {
        drive->cmdDrive(0_ft, 10_ft, 180_deg, {});

        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 2) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 2)) {
            step++;
        }
    }
}

void Autonomous::centerTwoBall() {
    if(step == 0) {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 1) {
        drive->cmdDrive(0_ft, 10_ft, 180_deg, {});

        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 2) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 2)) {
            step++;
        }
    }
}

void Autonomous::rightTwoBall() {
    if(step == 0) {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 1) {
        drive->cmdDrive(0_ft, 10_ft, 180_deg, {});

        step++;
    }
    else if (step == 1 && drive->cmdIsFinished()) {
        step++;
    }
    else if (step == 2) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 2)) {
            step++;
        }
    }
}

void Autonomous::centerThreeBall() {
    if(step == 3) {
        centerTwoBall();
        step++;
    }
    else if(step == 4) {
        drive->cmdDrive(5_ft, 15_ft, 30_deg, {});
        step++;
    }
    else if(step == 5 && drive->cmdIsFinished()){
        step++;
    }
    else if(step == 6) {
        drive->cmdDrive(5_ft, 15_ft, 150_deg, {});
        step++;
    }
    else if(step == 7 && drive->cmdIsFinished()){
        step++;
    }
    else if (step == 8) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 2)) {
            step++;
        }
    }

}

void Autonomous::rightShortThreeBall() {
    //hi peter p. lilley iii
    if(step == 3) {
        rightTwoBall();
        step++;
    }
    else if(step == 4) {
        drive->cmdDrive(5_ft, -5_ft, 45_deg, {});
        step++;
    }
    else if(step == 5 && drive->cmdIsFinished()){
        step++;
    }
    else if(step == 6) {
       drive->cmdDrive(.5_ft, 0_ft, -45_deg, {});
       step++;
    }
    else if(step == 7 && drive->cmdIsFinished()){
        step++;
    } 
    else if(step == 8) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 1)) {
            step++;
        }
    }
}

void Autonomous::rightFarThreeBall() {
    if(step == 3) {
        rightTwoBall();
        step++;
    }
    else if(step == 4) {
        drive->cmdDrive(5_ft, 15_ft, 150_deg, {});
        step++;
    }
    else if(step == 5 && drive->cmdIsFinished()){
        step++;
    }
    else if(step == 6) {
        drive->cmdDrive(10_ft, 10_ft, -100_deg, {});
        step++;
    }
    else if(step == 7 && drive->cmdIsFinished()){
        step++;
    }
    else if(step == 8) {
        if (alignAndShoot(Shooter::TARMAC_LINE, 1)) {
            step++;
        }
    }
}

void Autonomous::rightFourBall() {
    //hi nadia
    if(step == 9) {
        rightShortThreeBall();
        step++;
    }
    else if(step == 10) {
        drive->cmdDrive(10_ft, -3_ft, 100_deg, {});
        step++;
    }
    else if(step == 11 && drive->cmdIsFinished()){
        step++;
    }
    else if(step == 12) {
        drive->cmdDrive(10_ft, 10_ft, -100_deg, {});
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

bool Autonomous::alignAndShoot(Shooter::ShooterMode shooterMode, unsigned ballNum) {
    if (shootStep == 0) {
        // Start a command to align with high hub, and check if found by limelight.
        if (drive->cmdAlignToHighHub()) {
            shootStep++;
        }
        else {
            // Abort mission when high hub not found.
            currentMode = DO_NOTHING;
        }
    }
    else if (shootStep == 1) {
        //gamEpiece->shootABall(shooterMode);
        shootStep++;
    }
    else if (shootStep == 1/* && !gamEpiece->isShotInProgress()*/) {
        shootStep = 0;
        return true;
    }

    return false;
}

//                                       --- CAUTION ---
// What lies below is ishan's weird code that he did before we planned anything. Proceed with immense caution!
    //trevor