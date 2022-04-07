#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include "Feedback.h"
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <array>

// number of leds on each side.
#define LED_NUM_HANGER 80

class BlinkyBlinky : public Mechanism {
public:
    BlinkyBlinky();
    ~BlinkyBlinky();

    void resetToMode(MatchMode mode);
    void process() override;
    void sendFeedback() override;

    enum LEDMode {
        ALLIANCE,
        HANGER_STATUS,
        HOME_DEPOT,
        CRATER_MODE,
        DISABLED,
    };

    void setLEDMode(LEDMode mode);

private:
    frc::AddressableLED strip { PWM_BLINKY_BLINKY_STRIP };
    // frc::AddressableLED rightStrip { PWM_BLINKY_BLINKY_RIGHT };

    std::array<frc::AddressableLED::LEDData, LED_NUM_HANGER> stripBuffer {};
    std::array<frc::AddressableLED::LEDData, LED_NUM_HANGER> redColorRange {};
    std::array<frc::AddressableLED::LEDData, LED_NUM_HANGER> blueColorRange {};

    void setPixel(int index, frc::Color color);
    void setColor(frc::Color color);

    LEDMode ledMode = ALLIANCE;
    double redVal = 0;
    double greenVal = 0;
    double blueVal = 0;

    unsigned long long offset = 0;
};