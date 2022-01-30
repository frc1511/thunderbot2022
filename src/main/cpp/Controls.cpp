#include "Controls.h"

Controls::Controls(Drive* drive, GamEpiece* gamEpiece, Hang* hang) 
: drive(drive), gamEpiece(gamEpiece), hang(hang) {

}

Controls::~Controls() {

}

void Controls::process() {
    /*if(extendHangY == true && retractHangA == false){ // Extends hang if isnt retracting
         hang->move(Hang::UP);
     }
     // Retracts hang if button pressed
     else if(retractHangA == true && extendHangY == false){ // Retracts hang if isnt extending
         hang->move(Hang::DOWN);
     }
     else{ // If both pressed or neither pressed it does nothing
         hang->move(Hang::STOP);
     }*/
     bool isFieldCentric; // true=field centric  false=robot centric
    
}