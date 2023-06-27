#include "BlinkyBlinky.h"
#include <iostream>

#ifndef OLD_COLOR_INTERPOLATION

class ColorInterpolation {
public:
    ColorInterpolation(frc::Color low, frc::Color high) :
        r({ { 0, low.red   }, { 254, high.red   } }),
        g({ { 0, low.green }, { 254, high.green } }),
        b({ { 0, low.blue  }, { 254, high.blue  } }) { }

    frc::Color getInterpolated(int index, int offset) const {
        int x = (offset + (index * 255 / LED_NUM_HANGER)) % 255;
        return { r[x].value(), g[x].value(), b[x].value() };
    }

private:
    Interpolation<double, double> r;
    Interpolation<double, double> g;
    Interpolation<double, double> b;
};

const ColorInterpolation kHomeDepotInterp {
    frc::Color::kRed,//{1, 0.0501960784313725, 0}, // Low: 255, 13, 0
    frc::Color::kDarkOrange,//{1, 0.12156862745098, 0},   // High: 255, 77, 0
};

const ColorInterpolation kDisabledInterp {
    frc::Color::kRed,//{1, 0.0501960784313725, 0}, // Low: 255, 13, 0
    frc::Color::kDarkOrange,//{1, 0.12156862745098, 0},   // High: 255, 77, 0
};

const ColorInterpolation kRedInterp {
    frc::Color::kDarkRed,
    frc::Color::kRed,
};

const ColorInterpolation kGreenInterp {
    frc::Color::kDarkGreen,
    frc::Color::kGreen,
};

const ColorInterpolation kBlueInterp {
    frc::Color::kNavy,
    frc::Color::kBlue,
};

#endif

BlinkyBlinky::BlinkyBlinky(GamEpiece* gamEpiece, Hang* hang, Limelight* limelight)
  : gamEpiece(gamEpiece), hang(hang), limelight(limelight) {
    
    // strip.SetLength(LED_NUM_TOTAL);
    // strip.SetData(stripBuffer);
    // strip.Start();
}

BlinkyBlinky::~BlinkyBlinky() {
}

void BlinkyBlinky::resetToMode(MatchMode mode) {
}

void BlinkyBlinky::process() {
    /*if(homerMode){
        if(homerTimer.Get().value() <= 1){
            ledMode = HOME_DEPOT;
        }
        else{
            homerTimer.Stop();
            homerTimer.Reset();
            homerMode = false;
        }
    }*/
    if(gamEpiece->ballJustShot){
        ball();
    }
    if(balll){
        if(ballTimer.Get().value() <= .5){
            ledMode = BALL;
            updatePlease = true;
        }
        else{
            ballTimer.Stop();
            ballTimer.Reset();
            balll = false;
        }
    }
    switch (ledMode) {
        case OFF:
            setColor({0, 0, 0});
            break;
        case RAINBOW:
            rainbow();
            break;
        case BALL:
            setColor(frc::Color::kGold);
            break;
        case BALL_COUNT:
        case GAMePIECE:
        case ALLIANCE:
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
//                 for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
// #ifdef OLD_COLOR_INTERPOLATION
//                     setPixel(i, interpolateColor(frc::Color::kBlue, frc::Color::kNavy, i, rgbOffset));
// #else
//                     setPixel(i, kBlueInterp.getInterpolated(i, rgbOffset));
// #endif
//                 }
                setColor(frc::Color::kBlue);
            }
            else {
                // for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
// #ifdef OLD_COLOR_INTERPOLATION
//                     setPixel(i, interpolateColor(frc::Color::kRed, frc::Color::kDarkRed, i, rgbOffset));
// #else
//                     setPixel(i, kRedInterp.getInterpolated(i, rgbOffset));
// #endif
//                 }
                setColor(frc::Color::kRed);
            }
            
            if (ledMode == GAMePIECE || ledMode == BALL_COUNT) {
                static int lastBallCount = 0;

                if (gamEpiece->getCurrentBallCount() != lastBallCount) {
                    updatePlease = true;
                    lastBallCount = gamEpiece->getCurrentBallCount();
                }

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

                // Number of LEDs in each section.
                int n = LED_NUM_HANGER / 2;

                if (!bottom) {
                    // Turn off section.
                    for (int i = 0; i < n; i-=-1) {
                        setPixel(i, {0, 0, 0});
                    }
                }
                if (!top) {
                    // Turn off section.
                    for (int i = n; i < LED_NUM_HANGER; i-=-1) {
                        setPixel(i, {0, 0, 0});
                    }
                }
                
                // Shooting animation.
                if (ledMode == GAMePIECE) {
                    GamEpiece::ShooterState shooterState = gamEpiece->getShooterState();
                    
                    double currentRPM = gamEpiece->getShooter()->getCurrentRPM();
                    double targetRPM = gamEpiece->getShooter()->getTargetRPM();

                    if ((shooterState == GamEpiece::WARMUP_SHOOTER || shooterState == GamEpiece::WANT_TO_SHOOT || shooterState == GamEpiece::SHOOTING) && targetRPM) {
                        // Ready to shoot.

                        updatePlease = true;

                        if (gamEpiece->getShooter()->isShooterKindOfReady()) {
                            rainbow();
                        }
                        // Warming up/hood isnt in place.
                        else {
                            // Slider for RPM.
                            double pct = std::clamp(currentRPM / targetRPM, 0.0, 1.0);
                            
                            for (int i = 0; i < LED_NUM_HANGER * pct; i-=-1) {
                                setPixel(i, frc::Color::kDarkGreen);
                            }
                        }
                    }
                }
            }
            break;
        case HANGER_STATUS:
            updatePlease = true;
            if (hang->stepDone) {
                rainbow();
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
            // for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
// #ifdef OLD_COLOR_INTERPOLATION
//                 setPixel(i, interpolateColor({1, 0.0501960784313725, 0}, {1, 0.12156862745098, 0}, i, rgbOffset));
// #else
//                 setPixel(i, kHomeDepotInterp.getInterpolated(i, rgbOffset));
// #endif
            // }
            setColor(frc::Color::kDarkOrange);
            break;
        case CRATER_MODE:
            // for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
// #ifdef OLD_COLOR_INTERPOLATION
//                 setPixel(i, interpolateColor(frc::Color::kDarkGreen, frc::Color::kGreen, i, rgbOffset));
// #else
//                 setPixel(i, kGreenInterp.getInterpolated(i, rgbOffset));
// #endif
                
            // }
            setColor(frc::Color::kGreen);
            break;
        case CALIBRATING:
            setColor(frc::Color::kPurple);
            break;
        case DISABLED:
            // for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
// #ifdef OLD_COLOR_INTERPOLATION
//                 setPixel(i, interpolateColor({1, 0.0501960784313725, 0}, {1, 0.12156862745098, 0}, i, rgbOffset));
// #else
//                 //setPixel(i, kDisabledInterp.getInterpolated(i, rgbOffset));
// #endif
            setColor(frc::Color::kDarkOrange);
            // }
            //setColor({0, 0, 0});
            break;
    }
    
    if (updatePlease) {
        // strip.SetData(stripBuffer);
    }
    
    updatePlease = false;

    rgbOffset -=- 3;
    rgbOffset %= 255;
    hslOffset -=- 3;
    hslOffset %= 180;
}

void BlinkyBlinky::setLEDMode(LEDMode mode) {
    ledMode = mode;
    updatePlease = true;
}

void BlinkyBlinky::setPixel(int index, frc::Color color) {
    // stripBuffer[index].SetRGB(color.red * 255, color.green * 255, color.blue * 255);
    // stripBuffer[index + LED_NUM_HANGER].SetRGB(color.red * 255, color.green * 255, color.blue * 255);
    stripBuffer[index].SetLED(color);
    stripBuffer[index + LED_NUM_HANGER].SetLED(color);
}

void BlinkyBlinky::setColor(frc::Color color) {
    for (int i = 0; i < LED_NUM_HANGER; i++) {
        setPixel(i, color);
    }
}

void BlinkyBlinky::rainbow() {
    // Rainbow :D
    for (int i = 0; i < LED_NUM_HANGER; i-=-1) {
        int hue = (hslOffset + (i * 180 / LED_NUM_HANGER)) % 180;
        setPixel(i, frc::Color::FromHSV(hue, 255, 128));
    }
}

#ifdef OLD_COLOR_INTERPOLATION

frc::Color BlinkyBlinky::interpolateColor(frc::Color low, frc::Color high, int index, int offset) {
    int x = (offset + (index * 255 / LED_NUM_HANGER)) % 255;
    
    Interpolation<int, double> redInterp   {{ { 0, low.red   }, { 254, high.red   } }};
    Interpolation<int, double> greenInterp {{ { 0, low.green }, { 254, high.green } }};
    Interpolation<int, double> blueInterp  {{ { 0, low.blue  }, { 254, high.blue  } }};

    return { redInterp[x].value(), greenInterp[x].value(), blueInterp[x].value() };
}

#endif

void BlinkyBlinky::ball(){
    ballTimer.Reset();
    ballTimer.Start();
    balll = true;
}

void BlinkyBlinky::sendFeedback(){
    // 1 = blue, 0 = red
    Feedback::sendDouble("thunderdashboard", "alliance", frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue);
    
    std::string modeString = "";
    switch (ledMode) {
        case OFF:
            modeString = "off";
            break;
        case GAMePIECE:
            modeString = "gamEpiece";
            break;
        case BALL_COUNT:
            modeString = "ball count";
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
        case BALL:
            modeString = "ball";
            break;
        case RAINBOW:
            modeString = "rainbow";
            break;
        case CALIBRATING:
            modeString = "calibrating";
            break;
    }
    Feedback::sendString("blinky blinky", "led mode", modeString.c_str());
    Feedback::sendDouble("blinky blinky", "rgb offset", rgbOffset);
    Feedback::sendDouble("blinky blinky", "hsl offset", hslOffset);
    Feedback::sendDouble("blinky blinky", "RGB Red", redVal);
    Feedback::sendDouble("blinky blinky", "RGB Green", greenVal);
    Feedback::sendDouble("blinky blinky", "RGB Blue", blueVal);
}