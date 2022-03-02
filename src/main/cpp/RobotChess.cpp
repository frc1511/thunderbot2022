#include "RobotChess.h"


RobotChess::RobotChess(){
    // hi ishan
    std::ifstream isItHomer1;
    std::ifstream isItTestBoard;
    std::ifstream isItHomer2;
    isItHomer1.open("/home/lvuser/homer1");
    if(isItHomer1){
        #if !defined(HOMER) || defined(TEST_BOARD) // crash if homner isnt defined or testboard is
            std::cout << "boom\n";
            exit(-1);
        #endif
    }
    isItHomer1.close();
    isItTestBoard.open("/home/lvuser/testBoard"); // crash if testboard isnt defined or homer is
    if(isItTestBoard){
        #if !defined(TEST_BOARD) || defined(HOMER)
            std::cout << "boom\n";
            exit(-1);
        #endif

    }
    isItTestBoard.close();
    isItHomer2.open("/home/lvuser/homer2");
    if(isItHomer2){
        #if defined(HOMER) || defined(TEST_BOARD) // crash if testboard or homer is defined
            std::cout << "boom\n";
            exit(-1);
        #endif

    }
    isItHomer2.close();
    


}

RobotChess::~RobotChess(){
    // hi jeff. this was peters idea im sorry
}