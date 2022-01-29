#include "Autonomous.h"

#define VISION_ROTATE_SPEED .1

Autonomous::Autonomous(Drive* drive, GamEpiece* gamEpiece, Camera* camera) 
  : drive(drive), gamEpiece(gamEpiece), camera(camera) {
    
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
    handleDashboardString(FIVE_BALL, "(Positioned center) Score ball in robot, ball behind robot, ball at alliance station wall, and 2 balls at cargo line", buffer);

    Feedback::sendString("thunderdashboard", "auto_list", buffer);
}

void Autonomous::process() {
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
        case FIVE_BALL:
            fiveBall();
            break;
    }
}

void Autonomous::doNothing() {
    // If it does nothing is it doing something or nothing?
    // Good function.
}

void Autonomous::oneBall() {

}

void Autonomous::leftTwoBall() {

}

void Autonomous::centerTwoBall() {

}

void Autonomous::rightTwoBall() {

}

void Autonomous::leftThreeBall() {
    // hi jeff :D
}

void Autonomous::centerThreeBall() {

}

void Autonomous::rightThreeBall() {

}

void Autonomous::fiveBall() {

}

bool Autonomous::rotateToCargo() {
    camera->getFrame(&frame);

    switch (camera->locateTarget(frame)) {
        case Camera::UNKNOWN:
            // Infinitely loop..............................
            break;
        case Camera::CENTER:
            // Stop the drive.
            drive->manualDrive(0, 0, 0);
            return true;
        case Camera::LEFT:
            // Begin rotating to the left.
            drive->manualDrive(0, 0, -VISION_ROTATE_SPEED);
            break;
        case Camera::RIGHT:
            // Begin rotating to the right.
            drive->manualDrive(0, 0, +VISION_ROTATE_SPEED);
            break;
    }

    return false;
}