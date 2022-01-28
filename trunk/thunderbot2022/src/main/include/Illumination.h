#pragma once

#include "IOMap.h"
#include "Mechanism.h"
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <array>

#define LED_NUM_UNDERGLOW_FRONT 35
#define LED_NUM_UNDERGLOW_BACK 35
#define LED_NUM_UNDERGLOW_LEFT 45
#define LED_NUM_UNDERGLOW_RIGHT 45 

#define LED_NUM_UNDERGLOW_TOTAL (LED_NUM_UNDERGLOW_FRONT + LED_NUM_UNDERGLOW_BACK + LED_NUM_UNDERGLOW_LEFT + LED_NUM_UNDERGLOW_RIGHT)

#define LED_NUM_TOTAL (LED_NUM_UNDERGLOW_TOTAL)

class Illumination : public Mechanism {
public:
    Illumination();
    ~Illumination();

    void resetToMode(MatchMode mode);
    void process() override;

private:
    frc::AddressableLED strip { PWM_ILLUMINATION };
    std::array<frc::AddressableLED::LEDData, LED_NUM_TOTAL> stripBuffer {};
    std::array<frc::AddressableLED::LEDData, LED_NUM_TOTAL> colorRange {};

    enum LEDSection { UNDERGLOW_FRONT, UNDERGLOW_BACK, UNDERGLOW_LEFT, UNDERGLOW_RIGHT };

    void setPixel(LEDSection section, int index, frc::Color color);
    void setSection(LEDSection section, frc::Color color);    

    int underglowOffset = 0;
    frc::Timer timer {};
};