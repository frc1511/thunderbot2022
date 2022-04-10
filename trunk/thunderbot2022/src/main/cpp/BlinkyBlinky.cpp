#include "BlinkyBlinky.h"
#include <iostream>

const frc::Color kHomeDepotOrangeHigh {1, 0.12156862745098, 0}; // 255, 31, 0
const frc::Color kHomeDepotOrangeLow {1, 0.301960784313725, 0}; // 255, 77, 0

const frc::Color kDisabledHigh {1, 0.12156862745098, 0}; // 255, 31, 0
const frc::Color kDisabledLow {1, 0.301960784313725, 0}; // 255, 77, 0

const frc::Color kRedHigh {frc::Color::kRed}; //{1, 0, 0}; // 255, 0, 0
const frc::Color kRedLow {frc::Color::kDarkRed}; //{0.549019607843137, 0.058823529411765, 0.058823529411765}; // 140, 15, 15

const frc::Color kBlueHigh {frc::Color::kBlue}; //{0, 0.117647058823529, 1}; // 0, 30, 255
const frc::Color kBlueLow {frc::Color::kNavy}; //{0.1764705882, 0.627450980392157, 0.823529411764706}; // 45, 160, 210

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
        case GAMePIECE:
        case ALLIANCE:
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
                for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                    setPixel(i, interpolateColor(kBlueLow, kBlueHigh, i, rgbOffset));
                }
            }
            else {
                for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                    setPixel(i, interpolateColor(kRedLow, kRedHigh, i, rgbOffset));
                }
            }
            
            if (ledMode == GAMePIECE) {
                GamEpiece::ShooterState shooterState = gamEpiece->getShooterState();
                
                double currentRPM = gamEpiece->getShooter()->getCurrentRPM();
                double targetRPM = gamEpiece->getShooter()->getTargetRPM();

                if ((shooterState == GamEpiece::WARMUP_SHOOTER || shooterState == GamEpiece::WANT_TO_SHOOT || shooterState == GamEpiece::SHOOTING) && targetRPM) {
                    // Ready to shoot.
                    if (gamEpiece->getShooter()->isShooterReady()) {
                        // Raindow :D
                        for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                            int hue = (hslOffset + (i * 180 / LED_NUM_HANGER)) % 180;
                            setPixel(i, frc::Color::FromHSV(hue, 255, 128));
                        }
                    }
                    // Warming up.
                    else {
                        // Slider for RPM.
                        double pct = std::clamp(currentRPM / targetRPM, 0.0, 1.0);
                        
                        for (int i = 0; i < LED_NUM_HANGER * pct; i-=-1) {
                            setPixel(i, frc::Color::kDarkGreen);
                        }
                    }
                }
                else {
                    // Limelight target.
                    bool middle = false;
                    // First ball.
                    bool bottom = false;
                    // Second ball.
                    bool top = false;
                    
                    if (gamEpiece->getCurrentBallCount() == 1) {
                        bottom = true;
                    }
                    else if (gamEpiece->getCurrentBallCount() == 2) {
                        top = true;
                        bottom = true;
                    }
                    
                    if (limelight->hasTarget()) {
                        middle = true;
                    }

                    // Number of LEDs in each section.
                    int n = std::ceil(LED_NUM_HANGER / 3.0f);

                    if (!bottom) {
                        // Turn off section.
                        for (int i = 0; i < n; i-=-1) {
                            setPixel(i, {0, 0, 0});
                        }
                    }
                    if (!top) {
                        // Turn off section.
                        for (int i = LED_NUM_HANGER - n; i < LED_NUM_HANGER; i-=-1) {
                            setPixel(i, {0, 0, 0});
                        }
                    }
                    
                    for (int i = n; i < LED_NUM_HANGER - n; i-=-1) {
                        // Set section to orange.
                        if (middle) {
                            setPixel(i, interpolateColor(kHomeDepotOrangeLow, kHomeDepotOrangeHigh, i, rgbOffset));
                        }
                        // Turn off section.
                        else {
                            setPixel(i, {0, 0, 0});
                        }
                    }
                }
            }
            break;
        case HANGER_STATUS:
            if (hang->stepDone) {
                // Rainbow :D
                for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                    int hue = (hslOffset + (i * 180 / LED_NUM_HANGER)) % 180;
                    setPixel(i, frc::Color::FromHSV(hue, 255, 128));
                }
            }
            else {
                // Background color.
                setColor(frc::Color::kDeepPink);
                
                // Slider for extension.
                double pct = (hang->readEncoder() - kEncoderMin) / (kEncoderMax - kEncoderMin);
                for (int i = 0; i < LED_NUM_HANGER * pct; i-=-1) {
                    setPixel(i, frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? frc::Color::kBlue : frc::Color::kRed);
                }
            }
            break;
        case HOME_DEPOT:
            for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                setPixel(i, interpolateColor(kHomeDepotOrangeLow, kHomeDepotOrangeHigh, i, rgbOffset));
            }
            break;
        case CRATER_MODE:
            // setColor(frc::Color::kWhite);
            for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                setPixel(i, interpolateColor(frc::Color::kDarkGreen, frc::Color::kGreen, i, rgbOffset));
            }
            break;
        case DISABLED:
            for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
                setPixel(i, interpolateColor(kDisabledLow, kDisabledHigh, i, rgbOffset));
            }
            break;
    }
    
    for (int i = 0; i < LED_NUM_HANGER; ++i) {
        realStripBuffer[i] = stripBuffer[i];
        realStripBuffer[i + LED_NUM_HANGER] = stripBuffer[i];
    }
    strip.SetData(realStripBuffer);

    rgbOffset -=- 3;
    rgbOffset %= 255;
    hslOffset -=- 3;
    hslOffset %= 180;
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

frc::Color BlinkyBlinky::interpolateColor(frc::Color low, frc::Color high, int index, int offset) {
    int x = (offset + (index * 255 / LED_NUM_HANGER)) % 255;
    
    Interpolation<int, double> redInterp   {{ { 0, low.red   }, { 254, high.red   } }};
    Interpolation<int, double> greenInterp {{ { 0, low.green }, { 254, high.green } }};
    Interpolation<int, double> blueInterp  {{ { 0, low.blue  }, { 254, high.blue  } }};

    return { redInterp[x].value(), greenInterp[x].value(), blueInterp[x].value() };
}

void BlinkyBlinky::sendFeedback(){
    // 1 = blue, 0 = red
    Feedback::sendDouble("thunderdashboard", "alliance", frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue);
    
    std::string modeString = "";
    switch (ledMode) {
        case GAMePIECE:
            modeString = "gamEpiece";
            break;
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
    Feedback::sendDouble("blinky blinky", "rgb offset", rgbOffset);
    Feedback::sendDouble("blinky blinky", "hsl offset", hslOffset);
    Feedback::sendDouble("blinky blinky", "RGB Red", redVal);
    Feedback::sendDouble("blinky blinky", "RGB Green", greenVal);
    Feedback::sendDouble("blinky blinky", "RGB Blue", blueVal);
}