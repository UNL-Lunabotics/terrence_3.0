#include <Arduino.h>
#include <RoboClaw.h>

#define ROBOCLAW_BAUD (115200)
#define EXECAVATION_ROBOCLAW_ADDRESS (0x80)
#define DRIVETRAIN_ROBOCLAW_ADDRESS (0x82)

#define LEFT_MOTOR_PIN (6)
#define RIGHT_MOTOR_PIN (7)
#define HOPPER_MOTOR_PIN (8)
#define HOPPER_SERVO_PIN (29)

RoboClaw execavation_roboclaw(&Serial2, 10000);
RoboClaw drivetrain_roboclaw(&Serial3, 10000);

void setup() {
    Serial.begin(57600);
    execavation_roboclaw.begin(115200);
    drivetrain_roboclaw.begin(115200);
    drivetrain_roboclaw.ForwardMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, 0);
}

void loop() {

    while (Serial.available() == 0);  // wait for data
    String command = Serial.readString();  // read until timeout
    command.trim();  // remove tailing whitespace

    const char command_type = command.charAt(0);
    const char* c_str_command = command.c_str() + (sizeof(char) * 2);  // skip the command letter and space

    switch (command_type)
    {
        case 'f':
            uint8_t v; sscanf(c_str_command, "%u", &v);
            Serial.printf("[LOG]: RECEIVED MOTOR COMMAND :  FORWARD <= %d\n", v);
            drivetrain_roboclaw.ForwardMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
            break;
        
        case 'b':
            uint8_t v; sscanf(c_str_command, "%u", &v);
            Serial.printf("[LOG]: RECEIVED MOTOR COMMAND : BACKWARD <= %d\n", v);
            drivetrain_roboclaw.BackwardMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
            break;
        
        case 'l':
            uint8_t v; sscanf(c_str_command, "%u", &v);
            Serial.printf("[LOG]: RECEIVED MOTOR COMMAND :     LEFT <= %d\n", v);
            drivetrain_roboclaw.TurnLeftMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
            break;
        
        case 'r':
            uint8_t v; sscanf(c_str_command, "%u", &v);
            Serial.printf("[LOG]: RECEIVED MOTOR COMMAND :    RIGHT <= %d\n", v);
            drivetrain_roboclaw.TurnRightMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
            break;
        
        default:
            break;
    }
}
