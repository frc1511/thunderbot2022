#include "WooshWoosh.h"

WooshWoosh::WooshWoosh(){

}

WooshWoosh::~WooshWoosh(){

}

double WooshWoosh::getWoosh(WooshDirection direction){
    if(getWooshWoosh())
        return getWooshAngle(direction).value();
    else    
        return getWooshRate(direction).value();
}

units::radian_t WooshWoosh::getWooshAngle(WooshDirection direction){
    return units::radian_t(7);
}

units::radians_per_second_t WooshWoosh::getWooshRate(WooshDirection direction){
    return units::radians_per_second_t(123);
}

int WooshWoosh::getWooshWoosh(){
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(0,1);
    return dist6(rng);
}