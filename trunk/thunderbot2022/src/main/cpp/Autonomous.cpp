#include "Autonomous.h"

#define LEFT_START_POSITION (frc::Pose2d(0_ft, 0_ft, 0_deg))
#define CENTER_START_POSITION (frc::Pose2d(0_ft, 0_ft, 0_deg))
#define RIGHT_START_POSITION (frc::Pose2d(0_ft, 0_ft, 0_deg))

Autonomous::Autonomous(Drive* drive, GamEpiece* gamEpiece) 
  : drive(drive), gamEpiece(gamEpiece) {
    
}

Autonomous::~Autonomous() {

}

void Autonomous::resetToMode(MatchMode mode) {
    currentMode = DO_NOTHING;

    if (mode == MODE_AUTO) {
        currentMode = (AutoMode)Feedback::getEditableDouble("auto", "mode", 0);
        
        switch (currentMode) {
            case DO_NOTHING:
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

    handleDashboardString(LEFT_ONE_BALL,   "(Positioned left) Score ball in robot", buffer);
    handleDashboardString(CENTER_ONE_BALL, "(Positioned center) Score ball in robot", buffer);
    handleDashboardString(RIGHT_ONE_BALL,  "(Positioned right) Score ball in robot", buffer);

    handleDashboardString(LEFT_TWO_BALL,   "(Positioned left) Score ball 1 and ball in robot", buffer);
    handleDashboardString(CENTER_TWO_BALL, "(Positioned center) Score ball 2 and ball in robot", buffer);
    handleDashboardString(RIGHT_TWO_BALL,  "(Positioned right) Score ball 3 and ball in robot", buffer);
    //hi ishan
    handleDashboardString(CENTER_THREE_BALL,      "(Positioned center) Score ball 2, 4, and ball in robot", buffer);
    handleDashboardString(RIGHT_SHORT_THREE_BALL, "(Positioned right) Score ball 3, 2, and ball in robot", buffer);
    handleDashboardString(RIGHT_FAR_THREE_BALL,   "(Positioned right) Score ball 3, 4, and ball in robot", buffer);
    handleDashboardString(RIGHT_FOUR_BALL,        "(Positioned center) Score ball 3, 2, 4, and ball in robot", buffer);

    Feedback::sendString("thunderdashboard", "auto_list", buffer);
}

void Autonomous::process() {
    if (!gamEpiece)
        return;

    switch (currentMode) {
        case DO_NOTHING:
            doNothing();
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
    }
}

void Autonomous::doNothing() {
    // If it does nothing is it doing something or nothing?
    // Good function.
    // Very good function.
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
}

void Autonomous::centerOneBall() {

}

void Autonomous::rightOneBall() {

}

void Autonomous::leftTwoBall() {
    //hi calla
}

void Autonomous::centerTwoBall() {

}

void Autonomous::rightTwoBall() {

}

void Autonomous::centerThreeBall() {

}

void Autonomous::rightShortThreeBall() {

}

void Autonomous::rightFarThreeBall() {

}

void Autonomous::rightFourBall() {
    //hi nadia
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
        gamEpiece->shootABall(shooterMode);
        shootStep++;
    }
    else if (shootStep == 1 && !gamEpiece->isShotInProgress()) {
        shootStep = 0;
        return true;
    }

    return false;
}

// --- CAUTION ---
// What lies below is ishan's weird code that he did before we planned anything. Proceed with immense caution!

#if 0

    switch (currentMode) {
        case DO_NOTHING:
            doNothing();
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
        case LEFT_THREE_BALL:
            leftThreeBall();
            break;
        case CENTER_THREE_BALL:
            centerThreeBall();
            break;
        case RIGHT_THREE_BALL:
            rightThreeBall();
            break;
        case ALTERNATE_THREE_BALL:
            rightThreeBall();
            break;
        case FOUR_BALL:
            fourBall();
            break;
    }
}

#define AUTO_TRAJECTORY_CONFIG drive->getTrajectoryConfig()

void Autonomous::doNothing() {
    // If it does nothing is it doing something or nothing?
    // Good function.
    // Very good function.
}

void Autonomous::oneBall() {
    gamEpiece->shootABall(Shooter::TARMAC_LINE);
}

void Autonomous::leftTwoBall() {
    // rotateToCargo();
    step++;
    if(step == 1)
    {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 2)
    {
        frc::Trajectory leftTwoBallAutoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(leftTwoBallAutoTrajectory);
        step++;
    }
    else if(step == 3 && drive->cmdIsFinished() == true)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
    }
}

void Autonomous::centerTwoBall() {
    // rotateToCargo();
    step++;
    if(step == 1)
    {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 2)
    {
        frc::Trajectory centerTwoBallAutoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(centerTwoBallAutoTrajectory);
        step++;
    }
    else if(step == 3 && drive->cmdIsFinished() == true)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
    }
}

void Autonomous::rightTwoBall() {
    // rotateToCargo();
    step++;
    if(step == 1)
    {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 2)
    {
        frc::Trajectory rightTwoBallAutoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(rightTwoBallAutoTrajectory);
        step++;
    }
    else if(step == 3 && drive->cmdIsFinished() == true)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
        step++;
    }
}

void Autonomous::leftThreeBall() {
    leftTwoBall();
    if(step == 4)
    {
        // rotateToCargo();
        step++;
    }
    else if(step == 5)
    {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 6)
    {
        frc::Trajectory leftThreeBallAutoToWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(leftThreeBallAutoToWallTrajectory);
        step++;
    }
    else if(step == 7 && drive->cmdIsFinished() == true)
    {
        frc::Trajectory leftThreeBallAutoFromWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(leftThreeBallAutoFromWallTrajectory);
        step++;
    }
    else if(step == 8)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
        step++;
    }
}

void Autonomous::centerThreeBall() {
    centerTwoBall();
    if(step == 4)
    {
        // rotateToCargo();
        step++;
    }
    else if(step == 5)
    {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 6)
    {
        frc::Trajectory centerThreeBallAutoToWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(centerThreeBallAutoToWallTrajectory);
        step++;
    }
    else if(step == 7 && drive->cmdIsFinished() == true)
    {
        frc::Trajectory centerThreeBallAutoFromWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(centerThreeBallAutoFromWallTrajectory);
        step++;
    }
    else if(step == 8)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
        step++;
    }
}

void Autonomous::rightThreeBall() {
    rightTwoBall();
    if(step == 4)
    {
        // rotateToCargo();
        step++;
    }
    else if(step == 5)
    {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 6)
    {
        frc::Trajectory rightThreeBallAutoToWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(rightThreeBallAutoToWallTrajectory);
        step++;
    }
    else if(step == 7 && drive->cmdIsFinished() == true)
    {
        frc::Trajectory rightThreeBallAutoFromWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 5_ft, 6_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(rightThreeBallAutoFromWallTrajectory);
        step++;
    }
    else if(step == 8)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
        step++;
    }
}

void Autonomous::alternateThreeBall() {
    centerTwoBall();
    if(step == 4)
    {
        frc::Trajectory altThreeBallAutoRotateTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 0_ft, 0_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(altThreeBallAutoRotateTrajectory);
        step++;
    }
    else if(step == 5 && drive->cmdIsFinished() == true)
    {
        gamEpiece->setIntakeDirection(GamEpiece::INTAKE);
        step++;
    }
    else if(step == 6)
    {
        frc::Trajectory altThreeBallAutoToLeftTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 0_ft, 0_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(altThreeBallAutoToLeftTrajectory);
        step++;
    }
    else if(step == 7 && drive ->cmdIsFinished() == true)
    {
        frc::Trajectory altThreeBallAutoToWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 0_ft, 0_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(altThreeBallAutoToWallTrajectory);
        step++;
    }
    else if(step == 7 && drive ->cmdIsFinished() == true)
    {
        frc::Trajectory altThreeBallAutoFromWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 0_ft, 0_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(altThreeBallAutoFromWallTrajectory);
        step++;
    }
    else if(step == 8)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
    }
}

void Autonomous::fourBall() {
    leftTwoBall();
    if(step == 4)
    {
        frc::Trajectory fourBallAutoToWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 0_ft, 0_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(fourBallAutoToWallTrajectory);
        step++;
    }
    else if(step == 5 && drive -> cmdIsFinished() == true)
    {
        frc::Trajectory fourBallAutoFromWallTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            drive->getPose(), {}, { 0_ft, 0_ft, 45_deg }, AUTO_TRAJECTORY_CONFIG);
            drive->cmdFollowTrajectory(fourBallAutoFromWallTrajectory);
        step++;
    }
    else if(step == 6 && drive->cmdIsFinished() == true)
    {
        gamEpiece->shootABall(Shooter::TARMAC_LINE);
    }
}

#endif