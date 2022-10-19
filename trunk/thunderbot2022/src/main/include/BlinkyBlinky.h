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

// Uncomment if new way doesn't work when we get to houston.
// #define OLD_COLOR_INTERPOLATION

// Number of leds on each side.
#define LED_NUM_HANGER 38

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
        OFF,           // On or off, we don't know.
        RAINBOW,       // Cycling rainbow.
        BALL,          // Gold (after a ball was shot).
        ALLIANCE,      // The alliance color.
        BALL_COUNT,    // The ball count.
        GAMePIECE,     // The ball count and shooter rpm progress animation.
        HANGER_STATUS, // Hanger extention progress animation and rainbow when step finished.
        HOME_DEPOT,    // Orange.
        CRATER_MODE,   // Green.
        CALIBRATING,   // Purple.
        DISABLED,      // Idk something.
    };

    void setLEDMode(LEDMode mode);

    void ball();

private:
    GamEpiece* gamEpiece;
    Hang* hang;
    Limelight* limelight;

    frc::AddressableLED strip { PWM_BLINKY_BLINKY_STRIP };

    std::array<frc::AddressableLED::LEDData, LED_NUM_TOTAL> stripBuffer {};

    void setPixel(int index, frc::Color color);
    void setColor(frc::Color color);

    void rainbow();

#ifdef OLD_COLOR_INTERPOLATION
    frc::Color interpolateColor(frc::Color low, frc::Color high, int index, int offset);
#endif

    LEDMode ledMode = ALLIANCE;
    double redVal = 0;
    double greenVal = 0;
    double blueVal = 0;

    int rgbOffset = 0;
    int hslOffset = 0;

    bool updatePlease = true;

    frc::Timer ballTimer;
    bool balll = false;
};