#pragma once

/*
██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
██║██║   ██║    ██╔████╔██║███████║██████╔╝
██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     v2022
*/

//PWM-nothing right now

//CAN
#define CAN_HANG_WINCH_MOTOR 2
#define CAN_HANG_ENCODER_A 21
#define CAN_HANG_ENCODER_B 22

#define CAN_INTAKE_MOTOR 3

#define STORAGE_STAGE_ONE 4
#define STORAGE_STAGE_TWO 5

#define SHOOTER_TRANSITION_MOTOR 6
#define SHOOTER_LEFT_FLYWHEEL_MOTOR 7
#define SHOOTER_RIGHT_FLYWHEEL_MOTOR 8

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

//DIO
#define DIO_HANG_OPTICAL_HOME_SENSOR 0
#define DIO_HANG_FLAG_SENSOR_LEFT 1
#define DIO_HANG_FLAG_SENSOR_RIGHT 2

#define DIO_INTAKE_BANNER_ENTRANCE 3
#define DIO_STORAGE_BANNER_STAGE_ONE 4
#define DIO_STORAGE_BANNER_STAGE_TWO 5
#define DIO_SHOOTER_BANNER_LEFT_ROBOT 6

//ANALOG
#define ANALOG_SHOOTER_HOOD_POTENTIOMETER 0

//PCM1
#define PCM1_HANG_PIVOT_RETRACT 0
#define PCM1_HANG_PIVOT_EXTEND 1
#define PCM1_HANG_LEFT_BRAKE_PISTON_RETRACT 2
#define PCM1_HANG_LEFT_BRAKE_PISTON_EXTEND 3
#define PCM1_HANG_RIGHT_BRAKE_PISTON_RETRACT 4
#define PCM1_HANG_RIGHT_BRAKE_PISTON_EXTEND 5

#define INTAKE_PIVOT_EXTEND 6
#define INTAKE_PIVOT_RETRACT 7
//PCM1 FULL

//PCM2

//RELAY