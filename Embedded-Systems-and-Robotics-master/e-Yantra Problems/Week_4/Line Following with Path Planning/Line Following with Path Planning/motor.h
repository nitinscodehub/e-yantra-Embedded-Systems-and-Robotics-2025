/*
 * motor.h
 *
 * Created: 22-03-2020 0:16:24
 *  Author: Kotesh
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

// This function configures pins of ATmega328p to which pins of L293D Motor Driver IC is connected
void motors_pin_config(void);

// This function configures pins of ATmega328p to which pins of Servo is connected
void servo_pin_config(void);

// This function to configure left and right channel pins of the L293D Motor Driver IC for PWM
void pwm_pin_config(void);

// This function initializes servos angle according to the value given
void servo(int value);

// This function configures left and right channel pins of the L293D Motor Driver IC for PWM
void timer_pwm_init(void);

// This function controls the speed of both the motors of Firebird-V
void speed(unsigned char speed_l, unsigned char speed_r);

// This function makes the Firebird-V move forward
void motors_move_forward(void);

// This function makes the Firebird-V move backward
void motors_move_backward(void);

// This function makes the Firebird-V move left
void motors_move_left(void);

// This function makes the Firebird-V move right
void motors_move_right(void);

// This function makes the Firebird-V move soft_left
void motors_move_soft_left(void);

// This function makes the Firebird-V move soft_right
void motors_move_soft_right(void);

// This function makes the Firebird-V stop
void motors_stop(void);

#endif /* MOTOR_H_ */