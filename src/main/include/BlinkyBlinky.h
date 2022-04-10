#pragma once

#include "IOMap.h"
#include "GamEpiece.h"
#include "Hang.h"
#include "Limelight.h"
#include "Mechanism.h"
#include "Feedback.h"
#include "Interpolation.h"
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <array>

// Number of leds on each side.
#define LED_NUM_HANGER 40

// Total number of leds.
#define LED_NUM_TOTAL (LED_NUM_HANGER * 2)

class BlinkyBlinky : public Mechanism {
public:
    BlinkyBlinky(GamEpiece* gamEpiece, Hang* hang, Limelight* limelight);
    ~BlinkyBlinky();

    void resetToMode(MatchMode mode);
    void process() override;
    void sendFeedback() override;

    enum LEDMode {
        ALLIANCE,
        GAMePIECE,
        HANGER_STATUS,
        HOME_DEPOT,
        CRATER_MODE,
        DISABLED,
    };

    void setLEDMode(LEDMode mode);

private:
    GamEpiece* gamEpiece;
    Hang* hang;
    Limelight* limelight;

    frc::AddressableLED strip { PWM_BLINKY_BLINKY_STRIP };

    std::array<frc::AddressableLED::LEDData, LED_NUM_HANGER> stripBuffer {};
    std::array<frc::AddressableLED::LEDData, LED_NUM_TOTAL> realStripBuffer {};

    void setPixel(int index, frc::Color color);
    void setColor(frc::Color color);

    frc::Color interpolateColor(frc::Color low, frc::Color high, int index, int offset);

    LEDMode ledMode = ALLIANCE;
    double redVal = 0;
    double greenVal = 0;
    double blueVal = 0;

    int rgbOffset = 0;
    int hslOffset = 0;
};