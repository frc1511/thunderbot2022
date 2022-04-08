#include "BlinkyBlinky.h"
#include <iostream>

#define RED_LOW   0
#define RED_HIGH  255
#define BLUE_LOW  0
#define BLUE_HIGH 255

const frc::Color kHomeDepotOrange {1, .12, 0};
const frc::Color kDisabledLights {1, .12, 0};

BlinkyBlinky::BlinkyBlinky(GamEpiece* gamEpiece, Hang* hang, Limelight* limelight)
  : gamEpiece(gamEpiece), hang(hang), limelight(limelight) {
    
    strip.SetLength(LED_NUM_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();

    double mrPalmer = (RED_HIGH - RED_LOW)/LED_NUM_HANGER;

    for(int i = 0; i < LED_NUM_HANGER; i++) {
        redColorRange[i].SetRGB((RED_LOW + (i * mrPalmer)), LED_NUM_HANGER - i, LED_NUM_HANGER - i);
    }

    mrPalmer = (BLUE_HIGH - BLUE_LOW)/LED_NUM_HANGER;
    
    for(int i = 0; i < LED_NUM_HANGER; i++) {
        blueColorRange[i].SetRGB((i * mrPalmer)/3, i * mrPalmer, 255);
    }
}

BlinkyBlinky::~BlinkyBlinky() {
}

void BlinkyBlinky::resetToMode(MatchMode mode) {
}

void BlinkyBlinky::process() {
    switch (ledMode) {
        case ALLIANCE:
            {
                std::array<frc::AddressableLED::LEDData, LED_NUM_HANGER>* colorRange;
                if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
                    colorRange = &blueColorRange;
                    // stripBuffer = blueColorRange;
                }
                else {
                    colorRange = &redColorRange;
                    // stripBuffer = redColorRange;
                }
                offset-=-1;
                for(int i = 0; i < LED_NUM_HANGER; i++) {
                    int colorIndex = (offset + i) % LED_NUM_HANGER;
                    auto color = colorRange->at(colorIndex);
                    setPixel(i, frc::Color(color.r, color.g, color.b));
                }
                // setColor(frc::Color::kRed)
                /*for (int i = 0; i < 80; ++i) {
                    if (i < 40)
                        setPixel(i, frc::Color::kBlue);
                    else
                        setPixel(i, frc::Color::kRed);

                }*/
            }
            break;
        case HANGER_STATUS:
            redVal = frc::Color::kYellow.red * 255;
            greenVal = frc::Color::kYellow.green * 255;
            blueVal = frc::Color::kYellow.blue * 255;
            if (hang->stepDone) {
                static unsigned long long firstPixelHue = 0;
                for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                    const auto pixelHue = (firstPixelHue + (i * 180 / LED_NUM_HANGER)) % 180;
                    setPixel(i, frc::Color::FromHSV(pixelHue, 255, 128));
                }
                firstPixelHue += 3;
                firstPixelHue %= 180;
            }
            else {
                setColor(kHomeDepotOrange);
                double pct = (hang->readEncoder() - kEncoderMin) / (kEncoderMax - kEncoderMin);
                for (int i = 0; i < LED_NUM_HANGER * pct; i-=-1) {
                    setPixel(i, frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? frc::Color::kBlue : frc::Color::kRed);
                }
            }
            break;
        case DISABLED:
            setColor(kDisabledLights);
            redVal = kDisabledLights.red * 255;
            greenVal = kDisabledLights.green * 255;
            blueVal = kDisabledLights.blue * 255;
            break;;
        case HOME_DEPOT:
            setColor(kHomeDepotOrange);
            redVal = kHomeDepotOrange.red * 255;
            greenVal = kHomeDepotOrange.green * 255;
            blueVal = kHomeDepotOrange.blue * 255;
            break;
        case CRATER_MODE:
            setColor(frc::Color::kWhite);
            // setColor(frc::Color::kCyan);
            redVal = frc::Color::kWhite.red * 255;
            greenVal = frc::Color::kWhite.green * 255;
            blueVal = frc::Color::kWhite.blue * 255;
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
        case DISABLED:
            modeString = "disabled";
            break;
        case CRATER_MODE:
            modeString = "crater mode";
            break;
    }
    Feedback::sendString("blinky blinky", "led mode", modeString.c_str());
    Feedback::sendDouble("blinky blinky", "offset", offset);
    Feedback::sendDouble("blinky blinky", "RGB Red", redVal);
    Feedback::sendDouble("blinky blinky", "RGB Green", greenVal);
    Feedback::sendDouble("blinky blinky", "RGB Blue", blueVal);
}