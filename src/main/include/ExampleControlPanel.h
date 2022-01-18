#pragma once

// EXAMPLE FILE TO ILLUSTRATE AN ON-ROBOT MECHANISM

/*
 * PART 1: DESCRIBE MECHANISM
 * Actuators used, type of controller, and purpose:
 *     - Neo550 Motor, SparkMax
 *       Spins a wheel that positions the control panel field element
 * Sensors used and purpose
 *     - Rev Color sensor
 *       Looks down at control panel field element to
 *       determine currently active color
 *     - Encoder - internal encoder of the Neo550
 *       determines current spin speed of control panel wheel
 * Feedback to drivers
 *     - Color currently seen (visual display on dashboard)
 *     - Color currently seen using a giant light strip on the back of robot
 */

/*
 * PART 2: DESCRIBE COMMANDS AND ARGUMENTS
 * Normal commands:
 *     - Automatic Rotate
 *       Rotates the control panel some given number of full rotations
 *     - Automatic Position
 *       Turn the control panel to a specific color
 *     - Manual operation
 *       Turn the control panel wheel at a given speed and direction
 *       (does not use color sensor)
 * Information to support other mechanisms:
 *     - Way to indicate if an auto rotate or auto position command
 *       has completed the requested action
 *     - Way to expose currently seen color
 */



/******** PART 3: CODE FOR MECHANISM CLASS  ************/

#include "Mechanism.h"

class ExampleControlPanel : public Mechanism
{
public:
    /********** Supporting types **********/
    typedef enum {
        COLOR_RED,
        COLOR_BLUE,
        COLOR_YELLOW,
        COLOR_UNKNOWN
    } Color;

public:
    /******** COMMANDS ***********/
    /*
     * Start an automatic rotation of the control panel 
     * the given number of rotations.  Cancels any
     * in-progress command.
     */
    void autoRotate(int numberOfRotations);

    /*
     * Start an automatic positioning of the control panel 
     * to the specified color.  Cancels any
     * in-progress command.
     * The target color can be any color except UNKNOWN.
     */
    void autoPosition(Color targetColor);


    /*
     * Start manual rotation of the control panel 
     * at the specified speed and direction.  Cancels any
     * in-progress command.
     * Speed is -1 to 1, negative spins control panel counterclockwise,
     * positive spins clockwise
     */
    void manualRotate(double speed);

    /******** SUPPORT OF OTHER MECHANISMS ********/
    /*
     * Returns true if the last automatic command is complete,
     * false if it is not or if the last command was
     * for manual rotation.
     */
    bool isAutoComplete();

    /*
     * Returns the color currently seen by the color sensor
     */
    Color getCurrentColor();


public:
    /******** Methods of Mechanism class ***********/
    void sendFeedback() override;
    void process() override;

};
