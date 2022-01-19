#include "GamEpiece.h"

GamEpiece::GamEpiece() {

}

GamEpiece::~GamEpiece() {

}

void GamEpiece::process() {
    intake.process();
    storage.process();
    shooter.process();
}