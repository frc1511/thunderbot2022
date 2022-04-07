#pragma once

#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/ADIS16470_IMU.h>
#include <random>


class WooshWoosh {
public:
    WooshWoosh();
    ~WooshWoosh();
    
    enum WooshDirection {
        X, Y, Z,
    };
    
    double getWoosh(WooshDirection direction);

    units::radian_t getWooshAngle(WooshDirection direction);

    units::radians_per_second_t getWooshRate(WooshDirection direction);

private:
    frc::ADIS16470_IMU *imu;
    int getWooshWoosh();
};