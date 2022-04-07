#pragma once

#include "IOMap.h"

/*
██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
██║██║   ██║    ██╔████╔██║███████║██████╔╝
██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     v2022
*/

extern bool isCraterMode;

// #define HOMER // Comment out if not homer
 //#define TEST_BOARD // Enable for testing on test board

//CAN
#define CAN_HANG_WINCH_MOTOR 2

#define CAN_STORAGE_STAGE_ONE 3
#define CAN_STORAGE_STAGE_TWO 6

#define CAN_SHOOTER_LEFT_FLYWHEEL_MOTOR 7
#define CAN_SHOOTER_RIGHT_FLYWHEEL_MOTOR 8

#ifdef HOMER

#define CAN_SWERVE_FL_DRIVE_MOTOR 6
#define CAN_SWERVE_BL_DRIVE_MOTOR 8
#define CAN_SWERVE_BR_DRIVE_MOTOR 7
#define CAN_SWERVE_FR_DRIVE_MOTOR 5

#define CAN_SWERVE_FL_ROT_MOTOR 2
#define CAN_SWERVE_BL_ROT_MOTOR 4
#define CAN_SWERVE_BR_ROT_MOTOR 3
#define CAN_SWERVE_FR_ROT_MOTOR 1

#define CAN_SWERVE_FL_ROT_CAN_CODER 11
#define CAN_SWERVE_BL_ROT_CAN_CODER 12
#define CAN_SWERVE_BR_ROT_CAN_CODER 13
#define CAN_SWERVE_FR_ROT_CAN_CODER 10

#else

#define CAN_SWERVE_FL_DRIVE_MOTOR 9
#define CAN_SWERVE_BL_DRIVE_MOTOR 10
#define CAN_SWERVE_BR_DRIVE_MOTOR 11
#define CAN_SWERVE_FR_DRIVE_MOTOR 12

#define CAN_SWERVE_FL_ROT_MOTOR 13
#define CAN_SWERVE_BL_ROT_MOTOR 14
#define CAN_SWERVE_BR_ROT_MOTOR 15
#define CAN_SWERVE_FR_ROT_MOTOR 16

#define CAN_SWERVE_FL_ROT_CAN_CODER 17
#define CAN_SWERVE_BL_ROT_CAN_CODER 18
#define CAN_SWERVE_BR_ROT_CAN_CODER 19
#define CAN_SWERVE_FR_ROT_CAN_CODER 20

#endif


#ifndef TEST_BOARD

    //PWM
    #define PWM_HANG_RACHET_AND_PAWL 0
    #define PWM_SHOOTER_HOOD_SERVO 1
    #define PWM_BLINKY_BLINKY_STRIP 5

    //DIO
    #define DIO_HANG_OPTICAL_HOME_SENSOR 0

    #define DIO_STORAGE_BANNER_STAGE_ONE 2
    #define DIO_STORAGE_BANNER_STAGE_TWO 3
    #define DIO_SHOOTER_BANNER_LEFT_ROBOT 4
    #define DIO_HANG_REFLECTIVE_SENSOR 6
    #define DIO_STORAGE_BANNER_STAGE_HALF 5

    //ANALOG
    #define ANALOG_SHOOTER_HOOD_POTENTIOMETER 0

    //PCM1
    #define INTAKE_PIVOT_EXTEND_PISTON 0
    #define INTAKE_PIVOT_RETRACT_PISTON 1

    #define PCM1_HANG_PIVOT_1_EXTEND_PISTON 2
    #define PCM1_HANG_PIVOT_1_RETRACT_PISTON 3

    #define PCM1_HANG_PIVOT_2_EXTEND_PISTON 4
    #define PCM1_HANG_PIVOT_2_RETRACT_PISTON 5

    #define PCM1_HANG_STATIC_PISTON_RETRACT 6
    #define PCM1_HANG_STATIC_PISTON_EXTEND 7

#else // for test board

    //PWM
    #define PWM_HANG_RACHET_AND_PAWL 0          // Servo at edge of board near radio
    #define PWM_SHOOTER_HOOD_SERVO 1            // Loose servo; non-hi-tec 
    #define PWM_BLINKY_BLINKY 6                 // Not present
    // Missing on Wiki
    #define PWM_STRING_SERVO_RIGHT 7 
    #define PWM_STRING_SERVO_LEFT 8                 // Loose servo; hi-tec brand

    //DIO
    #define DIO_HANG_OPTICAL_HOME_SENSOR 5    // U-sensor floating on Rio
    #define DIO_HANG_ENCODER_A 8
    #define DIO_HANG_ENCODER_B 9

    #define DIO_STORAGE_BANNER_STAGE_ONE 0     // Big switch labelled 0.  Click for "beam broken". Requires INVERTED READ
    #define DIO_STORAGE_BANNER_STAGE_TWO 1      // Big switch labelled 1. Click for "beam broken". Requires INVERTED READ
    #define DIO_SHOOTER_BANNER_LEFT_ROBOT 3     // Large beam-break near off-board motor connectors. Requires INVERTED READ
    
    //ANALOG
    #define ANALOG_SHOOTER_HOOD_POTENTIOMETER 0 // Only pot on board

    //PCM1
    #define INTAKE_PIVOT_EXTEND_PISTON 3
    #define INTAKE_PIVOT_RETRACT_PISTON 4

    #define PCM1_HANG_PIVOT_1_EXTEND_PISTON 1
    #define PCM1_HANG_PIVOT_1_RETRACT_PISTON 6

    #define PCM1_HANG_PIVOT_2_EXTEND_PISTON 2
    #define PCM1_HANG_PIVOT_2_RETRACT_PISTON 5

    #define PCM1_HANG_BRAKE_PISTON_RETRACT 0    // Not present
    #define PCM1_HANG_BRAKE_PISTON_EXTEND 7     // Not present

#endif

//USB
#define USB_INTAKE_CAMERA 0