#include "Autonomous.h"

#define AUTO_TRAJECTORY_CONFIG drive->getTrajectoryConfig()

#define VISION_ROTATE_SPEED .1

Autonomous::Autonomous(Drive* drive, GamEpiece* gamEpiece) 
  : drive(drive), gamEpiece(gamEpiece) {
    
}

Autonomous::~Autonomous() {

}

void Autonomous::resetToMode(MatchMode mode) {
    currentMode = DO_NOTHING;

    if (mode == MODE_AUTO) {
        currentMode = (AutoMode)Feedback::getEditableDouble("auto", "mode", 0);
        timer.Reset();
        timer.Start();
        step = 0;
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
    
    handleDashboardString(DO_NOTHING, "doing nothing", buffer);
    handleDashboardString(ONE_BALL, "Score ball in robot", buffer);
    handleDashboardString(LEFT_TWO_BALL, "(Positioned left) Score ball in robot and ball behind robot", buffer);
    handleDashboardString(CENTER_TWO_BALL, "(Positioned center) Score ball in robot and ball behind robot", buffer);
    handleDashboardString(RIGHT_TWO_BALL, "(Positioned right) Score ball in robot and ball behind robot", buffer);
    //hi ishan
    handleDashboardString(LEFT_THREE_BALL, "(Positioned left) Score ball in robot, ball behind robot, ball at alliance station wall", buffer);
    handleDashboardString(CENTER_THREE_BALL, "(Positioned center) Score ball in robot, ball behind robot, ball at alliance station wall", buffer);
    handleDashboardString(RIGHT_THREE_BALL, "(Positioned right) Score ball in robot, ball behind robot, ball at alliance station wall", buffer);
    handleDashboardString(ALTERNATE_THREE_BALL, "(Positioned center) score ball in robot and behind robot, drive to left alliance partner ball and shoot", buffer);
    handleDashboardString(FOUR_BALL, "(Positioned center) Score ball in robot, ball behind robot, ball at alliance station wall, and 2 balls at cargo line", buffer);

    Feedback::sendString("thunderdashboard", "auto_list", buffer);
}

void Autonomous::process() {
    if (!gamEpiece)
        return;

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