#include "BlinkyBlinky.h"
#include <iostream>

const frc::Color kHomeDepotOrange {1, .12, 0};
const frc::Color kDisabledLights {1, .12, 0};

BlinkyBlinky::BlinkyBlinky(GamEpiece* gamEpiece, Hang* hang, Limelight* limelight)
  : gamEpiece(gamEpiece), hang(hang), limelight(limelight) {
    
    strip.SetLength(LED_NUM_TOTAL);
    strip.SetData(realStripBuffer);
    strip.Start();
}

BlinkyBlinky::~BlinkyBlinky() {
}

void BlinkyBlinky::resetToMode(MatchMode mode) {
}

void BlinkyBlinky::process() {
    switch (ledMode) {
        case ALLIANCE:
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
                for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                    static constexpr int redLow = 45;
                    static constexpr int greenLow = 160;
                    static constexpr int blueLow = 210;
                    
                    static constexpr int redHigh = 0;
                    static constexpr int greenHigh = 30;
                    static constexpr int blueHigh = 255;
                    
                    blueVal = (allianceOffset + (i * 255 / LED_NUM_HANGER)) % 255;
                    // Scale color into range.
                    blueVal = ((blueVal / 255) * (blueHigh - blueLow)) + blueLow;

                    static const Interpolation<int, int> redInterp   {{ { blueLow, redLow },   { blueHigh, redHigh }   }};
                    static const Interpolation<int, int> greenInterp {{ { blueLow, greenLow }, { blueHigh, greenHigh } }};

                    redVal = redInterp[blueVal].value();
                    greenVal = greenInterp[blueVal].value();
                    
                    setPixel(i, {redVal/255, greenVal/255, blueVal/255});
                }
            }
            else {
                for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                    static constexpr int redLow = 140;
                    static constexpr int greenLow = 15;
                    static constexpr int blueLow = 15;
                    
                    static constexpr int redHigh = 255;
                    static constexpr int greenHigh = 0;
                    static constexpr int blueHigh = 0;
                    
                    redVal = (allianceOffset + (i * 255 / LED_NUM_HANGER)) % 255;
                    // Scale color into range.
                    redVal = ((redVal / 255) * (redHigh - redLow)) + redLow;
                    
                    static const Interpolation<int, int> greenInterp {{ { redLow, greenLow }, { redHigh, greenHigh } }};
                    static const Interpolation<int, int> blueInterp  {{ { redLow, blueLow },  { redHigh, blueHigh }   }};

                    greenVal = greenInterp[redVal].value();
                    blueVal = blueInterp[redVal].value();
                    
                    setPixel(i, {redVal/255, greenVal/255, blueVal/255});
                }
            }
            allianceOffset -=- 3;
            allianceOffset %= 255;
            break;
        case HANGER_STATUS:
            if (hang->stepDone) {
                for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                    int hue = (hangerOffset + (i * 180 / LED_NUM_HANGER)) % 180;
                    setPixel(i, frc::Color::FromHSV(hue, 255, 128));
                }
                hangerOffset += 3;
                hangerOffset %= 180;
            }
            else {
                setColor(kHomeDepotOrange);
                double pct = (hang->readEncoder() - kEncoderMin) / (kEncoderMax - kEncoderMin);
                for (int i = 0; i < LED_NUM_HANGER * pct; i-=-1) {
                    setPixel(i, frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? frc::Color::kBlue : frc::Color::kRed);
                }
            }
            break;
        case HOME_DEPOT:
            setColor(kHomeDepotOrange);
            break;
        case CRATER_MODE:
            setColor(frc::Color::kWhite);
            // setColor(frc::Color::kPink);
            redVal = frc::Color::kWhite.red * 255;
            greenVal = frc::Color::kWhite.green * 255;
            blueVal = frc::Color::kWhite.blue * 255;
            break;
        case DISABLED:
            setColor(kDisabledLights);
            break;
    }
    
    for (int i = 0; i < LED_NUM_HANGER; ++i) {
        realStripBuffer[i] = stripBuffer[i];
        realStripBuffer[i + LED_NUM_HANGER] = stripBuffer[i];
    }
    strip.SetData(realStripBuffer);
}

void BlinkyBlinky::setLEDMode(LEDMode mode) {
    ledMode = mode;
}

void BlinkyBlinky::setPixel(int index, frc::Color color) {
    stripBuffer[index].SetRGB(color.red * 255, color.green * 255, color.blue * 255);
}

void BlinkyBlinky::setColor(frc::Color color) {
    for (int i = 0; i < LED_NUM_HANGER; i++) {
        setPixel(i, color);
    }
}

void BlinkyBlinky::sendFeedback(){
    // 1 = blue, 0 = red
    Feedback::sendDouble("thunderdashboard", "alliance", frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue);
    std::string modeString = "";
    switch (ledMode) {
        case ALLIANCE:
            modeString = "alliance";
            break;
        case HANGER_STATUS:
            modeString = "hanger status";
            break;
        case HOME_DEPOT:
            modeString = "home depot";
            break;
        case CRATER_MODE:
            modeString = "crater mode";
            break;
        case DISABLED:
            modeString = "disabled";
            break;
    }
    Feedback::sendString("blinky blinky", "led mode", modeString.c_str());
    Feedback::sendDouble("blinky blinky", "alliance color offset", allianceOffset);
    Feedback::sendDouble("blinky blinky", "hanger color offset", hangerOffset);
    Feedback::sendDouble("blinky blinky", "RGB Red", redVal);
    Feedback::sendDouble("blinky blinky", "RGB Green", greenVal);
    Feedback::sendDouble("blinky blinky", "RGB Blue", blueVal);
}