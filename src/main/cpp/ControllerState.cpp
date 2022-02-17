#include "ControllerState.h"


// Xbox button maps
const int kAButton = 1; // GetRawButton() give bool
const int kBButton = 2; // GetRawButton() give bool
const int kXButton = 3; // GetRawButton() give bool
const int kYButton = 4; // GetRawButton() give bool
const int kLeftBumper = 5; // GetRawButton() give bool
const int kRightBumper = 6; // GetRawButton() give bool
const int kBackButton = 7; // GetRawButton() give bool
const int kStartButton = 8; // GetRawButton() give bool
const int kLeftTrigger = 2; // GetRawAxis() give float
const int kRightTrigger = 3; // GetRawAxis() give float
const int kLeftStickXAxis = 0; // GetRawAxis() give float
const int kLeftStickYAxis = 1; // GetRawAxis() give float
const int kRightStickXAxis = 4; // GetRawAxis() give float
const int kRightStickYAxis = 5; // GetRawAxis() give float
const int kDPad = 0; // GetPOV() give float

ControllerState::ControllerState(int controllerID):myController(controllerID){
    
}

ControllerState::~ControllerState(){

}
void ControllerState::reset(){
    normalOrRelay = true;
    std::cout << "reset :D\n";
    recordOrNot = false;
    buttons = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,};
    axes = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
}
void ControllerState::process(){
    if(!normalOrRelay){ //under autonomous control :D
        if(buttonsDone == false){
            if(buttonsTime.size() > 0){
                if(autoTimer.Get().value() >= buttonsTime[0]){ // the timer has surpassed the next time when somthing is toggled 
                    buttonsValues[buttonsInt[0]-1] = !buttonsValues[buttonsInt[0]-1]; //toggles the rihgt button, subtracts one cuase vectors start at 0 but buttons starts at 1
                    buttonsTime.erase(buttonsTime.begin());
                    buttonsInt.erase(buttonsInt.begin());
                }
            }
            else{
                std::cout << "buttons are done :D\n";
                buttonsDone = true;
            }
        }
        if(axisDone == false){
            if(axesTime.size() > 0){
                if(autoTimer.Get().value() >= axesTime[0]){
                    axesValues[axesInt[0]] = axesPos[0];
                    axesTime.erase(axesTime.begin());
                    axesInt.erase(axesInt.begin());
                    axesPos.erase(axesPos.begin());
               }
            }
            else{
                std::cout << "axis are done :D\n";
                axisDone = true;
            }
        }
        if(buttonsDone && axisDone){
            std::cout << "done :D\n";
            normalOrRelay = true;
        }
    }

    //records when each button is pressed and realeased
    for (int i = 0; i < 14; i++) {
        buttons[(2*i)+1] = buttons[2*i];
        buttons[2*i] = myController.GetRawButton(i+1);
        if(buttons[2*i] != buttons[(2*i)+1] && recordOrNot) { //the button IS pressed and WAS NOT pressed before this loop or the button IS NOT pressed and WAS pressed before this loop
            //std::cout << "which button:" << i << "pressed?" << buttons[2*i] << "\n";
            if(timerStartedYet == false){
                autoTimer.Reset();
                autoTimer.Start();
                timerStartedYet = true;
            }
            recordButton(i+1,autoTimer.Get().value());
        }
        
        if(i<=5){
            axes[2*i] = myController.GetRawAxis(i);
            if(fabs(axes[2*i]-axes[(2*i)+1]) >=.05 && recordOrNot){
                //std::cout << "which axis" << i << "where?" << axes[2*i] << "\n";
                if(timerStartedYet == false){
                    autoTimer.Reset();
                    autoTimer.Start();
                    timerStartedYet = true;
                }
                recordAxis(i,autoTimer.Get().value(), axes[2*i]);
                axes[(2*i)+1] = axes[2*i];
            }
        }
    }

}

bool ControllerState::getRawButton(int buttonID){
    //put stuff here
    return buttons[(buttonID-1)*2];
}

double ControllerState::getRawAxis(int axisID){ 
    //put stuff here
    return axes[axisID*2];
}

bool ControllerState::getRawButtonPressed(int buttonID){
    return buttons[(buttonID-1)*2] && !buttons[(buttonID*2)];
}

bool ControllerState::getRawButtonReleased(int buttonID){
    return !buttons[(buttonID-1)*2] && buttons[(buttonID*2)];
}

void ControllerState::setRawButton(int buttonID){
    //put stuff here
}

void ControllerState::setRawAxis(int axisID){
    //put stuff here
}

void ControllerState::recordButton(int whichButton, double time){
    std::ofstream AutoButtonsFile;
    AutoButtonsFile.open("/home/lvuser/autobuttons.txt", std::fstream::app);
    AutoButtonsFile << std::to_string(whichButton) << ", " << std::to_string(time) << "\n";
    AutoButtonsFile.close();
}

void ControllerState::recordAxis(int whichAxis, double time, double position){
    std::ofstream AutoAxesFile;
    AutoAxesFile.open("/home/lvuser/autoaxes.txt", std::fstream::app);
    AutoAxesFile << std::to_string(whichAxis) << ", " << std::to_string(time) << ", " << std::to_string(position) << "\n";
    AutoAxesFile.close();
}

void ControllerState::testStuff(){
    std::string testString = "";
    std::ifstream AutoAxesFile;
    AutoAxesFile.open("/home/lvuser/deploy/autoaxes.txt");
    if(!AutoAxesFile){
        std::cout << "errorrrrrrrrrrrrrrrrrrrrrrr\n";
    }
    while(getline(AutoAxesFile, testString)){
        
        std::cout << testString << "\n";
    }
    AutoAxesFile.close();
    testString = "";
    std::ifstream AutoButtonsFile;
    AutoAxesFile.open("/home/lvuser/autobuttons.txt");
    if(!AutoAxesFile){
        std::cout << "errorrrrrrrrrrrrrrrrrrrrrrr\n";
    }
    while(getline(AutoAxesFile, testString)){
        
        std::cout << testString << "\n";
    }
    AutoButtonsFile.close();
}

void ControllerState::clearAuto(){
    std::ofstream AutoButtonsFile("/home/lvuser/autobuttons.txt");
    AutoButtonsFile << "";
    AutoButtonsFile.close();
    std::ofstream AutoAxesFile("/home/lvuser/autoaxes.txt");
    AutoAxesFile << "";
    AutoAxesFile.close();
    //std::cout << "cleared :D\n";
}

void ControllerState::replayAuto(){
    std::string buttonsString;
    std::ifstream AutoButtonsFile("/home/lvuser/autobuttons.txt");
    while(getline(AutoButtonsFile, buttonsString)){
        buttonsInt.push_back(std::atoi(buttonsString.substr(0,1).c_str()));
        buttonsTime.push_back(std::atof(buttonsString.substr(3,8).c_str()));    
        }
    AutoButtonsFile.close();
    std::string axesString;
    std::ifstream AutoAxesFile("/home/lvuser/autoaxes.txt");
    
    while(getline(AutoAxesFile, axesString)){
        axesInt.push_back(std::atoi(axesString.substr(0,1).c_str()));
        axesTime.push_back(std::atof(axesString.substr(3,8).c_str()));
        axesPos.push_back(std::atof(axesString.substr(13,8).c_str()));
    }
    AutoAxesFile.close();
    autoTimer.Reset();
    autoTimer.Start();
    normalOrRelay = false;
    buttonsDone = false;
    axisDone = false;
    //std::cout << "phase 6 complete :D\n";
}

void ControllerState::record(){
    recordOrNot = !recordOrNot;
    if(recordOrNot){
        timerStartedYet = false;
    }
}

