#include "Illumination.h"

#define RED_LOW   150
#define RED_HIGH  255
#define BLUE_LOW  150
#define BLUE_HIGH 255

Illumination::Illumination() {
    timer.Start();;;;;;;;;;;;;;;;;;;
    // hi peter
    strip.SetLength(LED_NUM_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();
    double mrPalmer = LED_NUM_UNDERGLOW_TOTAL/(BLUE_HIGH - BLUE_LOW);
    for(int i = 0; i < LED_NUM_UNDERGLOW_TOTAL; i++) {
        colorRange [i].SetRGB(0, 0, BLUE_LOW + i * mrPalmer);
    }
}

Illumination::~Illumination() {
    timer.Stop();
}

void Illumination::resetToMode(MatchMode mode) {
    switch(mode) {
        case MODE_DISABLED:
            setSection(UNDERGLOW_FRONT, frc::Color(255, 0, 0));
            setSection(UNDERGLOW_RIGHT, frc::Color(255, 0, 0));
            setSection(UNDERGLOW_BACK,  frc::Color(255, 0, 0));
            setSection(UNDERGLOW_LEFT,  frc::Color(255, 0, 0));
            break;
        case MODE_AUTO:
            for(int i = 0; i < LED_NUM_UNDERGLOW_TOTAL; i++) {
                setPixel(UNDERGLOW_FRONT, i, frc::Color(colorRange [i].r, colorRange [i].g, colorRange [i].b));
            }
            break;
        case MODE_TELEOP:
            setSection(UNDERGLOW_FRONT, frc::Color(0, 255, 0));
            switch(frc::DriverStation::GetAlliance()) {
                case frc::DriverStation::kRed:
                    setSection(UNDERGLOW_RIGHT, frc::Color(255, 0, 0));
                    setSection(UNDERGLOW_BACK,  frc::Color(255, 0, 0));
                    setSection(UNDERGLOW_LEFT,  frc::Color(255, 0, 0));
                    break;
                case frc::DriverStation::kBlue:
                    setSection(UNDERGLOW_RIGHT, frc::Color(0, 0, 255));
                    setSection(UNDERGLOW_BACK,  frc::Color(0, 0, 255));
                    setSection(UNDERGLOW_LEFT,  frc::Color(0, 0, 255));
                    break;
                case frc::DriverStation::kInvalid:
                    break;
            }
            break;
        case MODE_TEST:
            setSection(UNDERGLOW_FRONT, frc::Color(255, 0, 0));
            setSection(UNDERGLOW_RIGHT, frc::Color(239, 245, 66));
            setSection(UNDERGLOW_BACK,  frc::Color(0, 255, 0));
            setSection(UNDERGLOW_LEFT,  frc::Color(0, 0, 255));
            break;
    }
}

void Illumination::process() {
    strip.SetData(stripBuffer);
    if(getCurrentMode() == MODE_AUTO) {
        if(timer.Get().value() > 0.1) {
            timer.Reset();
            underglowOffset += 1;
            for(int i = 0; i < LED_NUM_UNDERGLOW_TOTAL; i++) {
                int colorIndex = (underglowOffset + i) % LED_NUM_UNDERGLOW_TOTAL;
                setPixel(UNDERGLOW_FRONT, i, frc::Color(colorRange [colorIndex].r, colorRange [colorIndex].g, colorRange [colorIndex].b));
            }
        }
    }
}

void Illumination::setPixel(LEDSection section, int index, frc::Color color) {
    int offset = index;

    switch(section) {
        case UNDERGLOW_FRONT:
            break;
        case UNDERGLOW_RIGHT:
            offset += LED_NUM_UNDERGLOW_FRONT;
            break;
        case UNDERGLOW_BACK:
            offset += LED_NUM_UNDERGLOW_FRONT + LED_NUM_UNDERGLOW_RIGHT;
            break;
        case UNDERGLOW_LEFT:
            offset += LED_NUM_UNDERGLOW_FRONT + LED_NUM_UNDERGLOW_RIGHT + LED_NUM_UNDERGLOW_BACK;
            break;
    }

    stripBuffer[offset].SetRGB(color.red, color.green, color.blue);
}

void Illumination::setSection(LEDSection section, frc::Color color) {
    int length = 0;

    switch(section) {
        case UNDERGLOW_FRONT:
            length = LED_NUM_UNDERGLOW_FRONT;
            break;
        case UNDERGLOW_RIGHT:
            length = LED_NUM_UNDERGLOW_RIGHT;
            break;
        case UNDERGLOW_BACK:
            length = LED_NUM_UNDERGLOW_BACK;
            break;
        case UNDERGLOW_LEFT:
            length = LED_NUM_UNDERGLOW_LEFT;
            break;
    }

    for (int i = 0; i < length; i++) {
        setPixel(section, i, color);
    }
}