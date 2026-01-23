#ifndef __propulsionCmd_H
#define __propulsionCmd_H

#include <cstdint>
#include <stdint.h>


/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in MANUAL mode (from joystick orders)
*
* The joystick sends throttle order, which is directly transformed into PWM. The PWMs are equal for both motors in Manual Mode
* 
*/
int * manualPropulsionCmd(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd);
int * obstacleDetection1(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd,int us_front,int us_front_right,int us_front_left,int us_rear,int us_rear_right,int us_rear_left);
int * obstacleDetection2(float& requestedSteerAngle, float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd,int us_front,int us_front_right,int us_front_left,int us_rear,int us_rear_right,int us_rear_left);
float Steercalculation (int us_front_left, int us_front_right, int us_rear_right, int us_rear_left);
void autoBloque (int& is_stopped, int us_front_left, int us_front_right, int us_rear_right, int us_rear_left, int us_rear, float& requestedSteerAngle, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd );
float suivreHumain (int x, int w, float past_steer_angle);

#endif /*__ propulsionCmd_H */