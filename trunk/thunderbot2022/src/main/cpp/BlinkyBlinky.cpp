#include "BlinkyBlinky.h"

#define RED_LOW   150
#define RED_HIGH  255
#define BLUE_LOW  150
#define BLUE_HIGH 255

const frc::Color kHomeDepotOrange { .9333, .4431, .1451 };
const frc::Color kDisabledLights  { .9333, .4431, .1451 };

BlinkyBlinky::BlinkyBlinky() {
    strip.SetLength(LED_NUM_HANGER);
    //rightStrip.SetLength(LED_NUM_HANGER);
    strip.SetData(stripBuffer);
    //rightStrip.SetData(stripBuffer);
    strip.Start();
    //rightStrip.Start();

    double mrPalmer = LED_NUM_HANGER/(RED_HIGH - RED_LOW);

    for(int i = 0; i < LED_NUM_HANGER; i++) {
        redColorRange[i].SetRGB((RED_LOW + i * mrPalmer)/255, 0, 0);
    }

    mrPalmer = LED_NUM_HANGER/(BLUE_HIGH - BLUE_LOW);
    
    for(int i = 0; i < LED_NUM_HANGER; i++) {
        blueColorRange[i].SetRGB(0, 0, (BLUE_LOW + i * mrPalmer)/255);
    }
}

BlinkyBlinky::~BlinkyBlinky() {
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
}

void BlinkyBlinky::resetToMode(MatchMode mode) {
}

void BlinkyBlinky::process() {
    switch (ledMode) {
        case ALLIANCE:
            {
                /*
                std::array<frc::AddressableLED::LEDData, LED_NUM_HANGER>* colorRange;
                if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
                    colorRange = &blueColorRange;
                }
                else {
                    colorRange = &redColorRange;
                }
                offset += 1;
                for(int i = 0; i < LED_NUM_HANGER; i++) {
                    int colorIndex = (offset + i) % LED_NUM_HANGER;
                    auto color = colorRange->at(colorIndex);
                    setPixel(i, frc::Color(color.r, color.g, color.b));
                }*/
                // setColor(frc::Color::kRed)
                for (int i = 0; i < 80; ++i) {
                    if (i < 40)
                        setPixel(i, frc::Color::kBlue);
                    else
                        setPixel(i, frc::Color::kRed);

                }
            }
            break;
        case HANGER_STATUS:
            setColor(frc::Color::kYellow);
            break;
        case DISABLED:
            setColor(kDisabledLights);
            break;
        case HOME_DEPOT:
            setColor(kHomeDepotOrange);
            break;
        case CRATER_MODE:
            setColor(frc::Color::kWhite);
            // setColor(frc::Color::kCyan);
            break;
    }
    
    strip.SetData(stripBuffer);
    //rightStrip.SetData(stripBuffer);
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