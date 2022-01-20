#include "Hang.h"
#include "string"

Hang::Hang() {
reset();
}

Hang::~Hang() {}

void Hang::process() {
    switch(hangState)
    {
            case MID:
                break;
            case HIGH:
                break;
            case TRAVERSAL:
                break;
            case NOT_ON_BAR:
                break;
    }

}

void Hang::reset() {
    int step = 0;
    bool isOnBar = false;
}

void Hang::debug() {
    std::string stateString = "";
    switch(hangState)
    {
        case MID:
            stateString = "going to mid";
            break;
        case HIGH:
            stateString = "going to high";
            break;
        case TRAVERSAL:
            stateString = "going to traversal";
            break;
    }
}

void Hang::pivot() {
    //toggles hang pivot piston
    hangPivot.Toggle();
}

void Hang::retract() {

}

void Hang::reversePivot() {
//dont know how to do this yet, maybe slow exhaust. Check on Hang subteam later
}

void Hang::engageBrake() {
}

void Hang::disengageBrake() {

}