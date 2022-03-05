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
    whichController = controllerID;
}

ControllerState::~ControllerState(){

}
void ControllerState::reset(){
    if(whichController == 0){
        autoButtonsFile = driverAutoFiles[(whichMode)*2];
        autoAxesFile = driverAutoFiles[(whichMode*2)+1];
    }
    else if(whichController == 1){
        autoButtonsFile = auxAutoFiles[(whichMode)*2];
        autoAxesFile = auxAutoFiles[(whichMode*2)+1];
    }
    normalOrRelay = true;
    //std::cout << "reset :D\n";
    recordOrNot = false;
    //buttons = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,};
    buttons = std::vector<bool>(28, false);
    //axes = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    axes = std::vector<double>(14,0);
}
void ControllerState::process(){

    //records when each button is pressed and realeased
    for (int i = 0; i < 14; i++) { // 14 buttons, 6 axis, 1 pov
        buttons[(2*i)+1] = buttons[2*i];
        if(normalOrRelay){ // checks if it should look to the controller for the value of the button
            buttons[2*i] = myController.GetRawButton(i+1);
        }
        if(buttons[2*i] != buttons[(2*i)+1] && recordOrNot) { // the button changed its state and we are trying to record
            std::cout << "which button:" << i << "pressed?" << buttons[2*i] << "\n";
            recordButton(i+1,autoTimer.Get().value()); // loop starts at 0 but buttons start at 1 so add 1
        }
      
        if(i<=5){ // this will trigger when i is 0-5 so all 6 axes
            if(normalOrRelay){ // checks if it should look to the controller for the value of the axes
                axes[2*i] = myController.GetRawAxis(i); 
            }
            if(fabs(axes[2*i]-axes[(2*i)+1]) >=.05 && recordOrNot){ // the axis changed by .05 
                std::cout << "which axis" << i << "where?" << axes[2*i] << "\n";
                recordAxis(i,autoTimer.Get().value(), axes[2*i]);
                axes[(2*i)+1] = axes[2*i];
            }
        }
        else if(i == 6){ // .GetPOV() is treated as a seventh axis so when i is 6. programmers start at 0 idk why its dumb
            axes[(2*i)+1] = axes[2*i];
            if(normalOrRelay){ // checks if it should look to the controller for the value of the pov
                axes[2*i] = myController.GetPOV();
            }
            if(axes[2*i] != axes[(2*i)+1] && recordOrNot){ // the POV changed from what it once was
                std::cout << "which axis" << i << "where?" << axes[2*i] << "\n";
                recordAxis(i,autoTimer.Get().value(), axes[2*i]);
                
            }
        }
    }
    ////std::cout << std::string(autoButtonsFile) << autoTimer.Get().value() << "\n";
    if(!normalOrRelay){ //under autonomous control :D
        if(buttonsDone == false){
            if(buttonsTime.size() > 0){ // make sure there is more buttons to change
                if(autoTimer.Get().value() >= buttonsTime[0]){ // the timer has surpassed the next time when somthing is toggled 
                    buttons[(buttonsInt[0]-1)*2] = !buttons[(buttonsInt[0]-1)*2]; //toggles the rihgt button, subtracts one cuase vectors start at 0 but buttons starts at 1
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
            if(axesTime.size() > 0){ // make sure there are more axes to be changed
                if(autoTimer.Get().value() >= axesTime[0]){
                    axes[axesInt[0]*2] = axesPos[0];
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
    return buttons[(buttonID-1)*2] && !buttons[((buttonID-1)*2)+1]; // button is pressed but wasnt pressed before
}

bool ControllerState::getRawButtonReleased(int buttonID){
    return !buttons[(buttonID-1)*2] && buttons[((buttonID-1)*2)+1];
}

void ControllerState::recordButton(int whichButton, double time){
    if(whichButton != 1){   
    std::ofstream AutoButtonsFile;
    AutoButtonsFile.open(autoButtonsFile, std::fstream::app);
    AutoButtonsFile << std::to_string(whichButton) << ", " << std::to_string(time) << "\n";
    AutoButtonsFile.close();
    }
}

void ControllerState::recordAxis(int whichAxis, double time, double position){
    std::ofstream AutoAxesFile;
    AutoAxesFile.open(autoAxesFile, std::fstream::app);
    AutoAxesFile << std::to_string(whichAxis) << ", " << std::to_string(time) << ", " << std::to_string(position) << "\n";
    AutoAxesFile.close();
}

void ControllerState::testStuff(){
    std::string testString = "";
    std::ifstream AutoAxesFile;
    AutoAxesFile.open(autoAxesFile);
    if(!AutoAxesFile){
        //std::cout << "errorrrrrrrrrrrrrrrrrrrrrrr\n";
    }
    //std::cout << "Auto Axes:\n";
    while(getline(AutoAxesFile, testString)){
        
        //std::cout << testString << "\n";
    }
    AutoAxesFile.close();
    testString = "";
    std::ifstream AutoButtonsFile;
    AutoAxesFile.open(autoButtonsFile);
    if(!AutoAxesFile){
        //std::cout << "errorrrrrrrrrrrrrrrrrrrrrrr\n";
    }
    //std::cout << "AutoButtons:\n";
    while(getline(AutoAxesFile, testString)){
        
        //std::cout << testString << "\n";
    }
    AutoButtonsFile.close();
}

void ControllerState::clearAuto(){
    std::ofstream AutoButtonsFile(autoButtonsFile);
    AutoButtonsFile << "";
    AutoButtonsFile.close();
    std::ofstream AutoAxesFile(autoAxesFile);
    AutoAxesFile << "";
    AutoAxesFile.close();
    std::cout << "cleared :D\n";
}

void ControllerState::replayAuto(){
    std::string buttonsString;
    std::ifstream AutoButtonsFile(autoButtonsFile);
    while(getline(AutoButtonsFile, buttonsString)){
        buttonsInt.push_back(std::atoi(buttonsString.substr(0,1).c_str()));
        buttonsTime.push_back(std::atof(buttonsString.substr(3,8).c_str()));    
        }
    AutoButtonsFile.close();
    std::string axesString;
    std::ifstream AutoAxesFile(autoAxesFile);
    
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
        autoTimer.Reset();
        autoTimer.Start();
        timerStartedYet = true;
        std::cout << "recording :D\n";
    }
}

void ControllerState::chooseAutoMode(int whichAutoMode){
    whichMode = whichAutoMode;
}