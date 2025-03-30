/*
 * motor.c
 *
 * Created: 22-03-2020 0:14:52
 *  Author: Kotesh
 */ 

#include "simulation.h"				// Header file included that contains macro definitions essential for Virtual turtle
#include <util/delay.h>				// Standard AVR Delay Library
#include "motor.h"					//Header file included that included functions required for motors

#define sbit( reg, bit )	reg |= ( 1 << bit )			// Macro defined for Setting a bit of any register
#define cbit( reg, bit )	reg &= ~( 1 << bit )		// Macro defined for Clearing a bit of any register


/**
 * @brief      Function to configure pins of ATmega328P to which pins of L293D Motor Driver IC is connected
 */
void motors_pin_config(void) {

	// set motor pins as output
	
	sbit(motors_dir_ddr_reg,motors_RB_pin);
	sbit(motors_dir_ddr_reg,motors_RF_pin);
	sbit(motors_dir_ddr_reg,motors_LF_pin);
	sbit(motors_dir_ddr_reg,motors_LB_pin);
	
	// stop motors initially
	
	cbit(motors_dir_port_reg,motors_RB_pin);
	cbit(motors_dir_port_reg,motors_RF_pin);
	cbit(motors_dir_port_reg,motors_LF_pin);
	cbit(motors_dir_port_reg,motors_LB_pin);
		
	
}

/**
 * @brief      Function to configure pins of ATmega328p to which pins of Servo is connected
 */
void servo_pin_config(void) {

	// set servo pin as output
	sbit(servo_ddr_reg,servo_pin);
	
	// reset the servo initially
	cbit(servo_port_reg,servo_pin);		
	
}

/**
 * @brief      Function to initialize servo 
 *
 */
void servo(int value){

	ICR1=4999;
	OCR1B = value;
	_delay_ms(500);
	
}

/**
 * @brief      Function to configure left and right channel pins of the L293D Motor Driver IC for PWM
 */
void pwm_pin_config(void){
	
	// set left and right channel pin as output
	
	sbit(motors_pwm_ddr_reg,motors_pwm_R_pin);
	sbit(motors_pwm_ddr_reg,motors_pwm_L_pin);
	
	// enable left and right channel
	
	sbit(motors_pwm_port_reg,motors_pwm_R_pin);
	sbit(motors_pwm_port_reg,motors_pwm_L_pin);

}

//-------------------
/**
 * @brief      Function to initialize Timer 0 and Timer 2 in Phase Correct PWM mode for speed control of motors of Virtual Turtle
 *
 */
void timer_pwm_init(void) {

	// Configure for Phase Correct PWM
	// Set Pre scalar to 64
	TCCR0A_reg = 0x21; 
	TCCR0B_reg = 0x03; 
	TCCR1A_reg = 0x21;
	TCCR1B_reg = 0x03;
		
	// Initialize Timer/counter register as 0
	TCNT0L_reg = 0x00;
	
	// Initialize OCR registers as 0
	OCR0BL_reg = 0x00;
	
	// Configure for Phase Correct PWM
	// Set Pre scalar to 64
	TCCR2A_reg = 0x21; 
	TCCR2B_reg = 0x03;
	
	// Initialize Timer/counter register as 0
	TCNT2L_reg = 0x00;
	
	// Initialize OCR registers as 0
	OCR2BL_reg = 0x00;
	
}


/**
 * @brief      Function to control the speed of both the motors of Virtual Turtle
 *
 * @param[in]  speed_l   Left motor speed 0 to 255
 * @param[in]  speed_r  Right motor speed 0 to 255
 */
void speed(unsigned char speed_l, unsigned char speed_r){
	
	OCR2BL_reg = speed_l;
	OCR0BL_reg = speed_r;
}


/**
 * @brief      Function to make Virtual Turtle move forward.
 */
void motors_move_forward(void) {
	
	// Set RF and LF, reset RB and LB
	
	cbit(motors_dir_port_reg,motors_RB_pin);
	cbit(motors_dir_port_reg,motors_LB_pin);
	sbit(motors_dir_port_reg,motors_RF_pin);
	sbit(motors_dir_port_reg,motors_LF_pin);
				
}

/**
 * @brief      Function to make Virtual Turtle move backward.
 */
void motors_move_backward(void) {
	
	// Set RB and LB, reset RF and LF
	
	cbit(motors_dir_port_reg,motors_RF_pin);
	cbit(motors_dir_port_reg,motors_LF_pin);
	sbit(motors_dir_port_reg,motors_RB_pin);
	sbit(motors_dir_port_reg,motors_LB_pin);
				
}

/**
 * @brief      Function to make Virtual Turtle move left.
 */
void motors_move_left(void) {
	
	// Set RF and LB, reset RB and LF
	
	cbit(motors_dir_port_reg,motors_RB_pin);
	cbit(motors_dir_port_reg,motors_LF_pin);
	sbit(motors_dir_port_reg,motors_RF_pin);
	sbit(motors_dir_port_reg,motors_LB_pin);
				
}

/**
 * @brief      Function to make Virtual Turtle move right.
 */
void motors_move_right(void) {
	
	// Set RB and LF, reset RF and LB
	
	cbit(motors_dir_port_reg,motors_RF_pin);
	cbit(motors_dir_port_reg,motors_LB_pin);
	sbit(motors_dir_port_reg,motors_RB_pin);
	sbit(motors_dir_port_reg,motors_LF_pin);
				
}

/**
 * @brief      Function to make Virtual Turtle move soft_left.
 */
void motors_move_soft_left(void) {
	
	// Set RF, reset RB,LF and LB
	
	cbit(motors_dir_port_reg,motors_LF_pin);
	cbit(motors_dir_port_reg,motors_LB_pin);
	cbit(motors_dir_port_reg,motors_RB_pin);
	sbit(motors_dir_port_reg,motors_RF_pin);
				
}

/**
 * @brief      Function to make Virtual Turtle move soft_right.
 */
void motors_move_soft_right(void) {
	
	// Set LF, reset RB,RF and LB
	
	cbit(motors_dir_port_reg,motors_RF_pin);
	cbit(motors_dir_port_reg,motors_LB_pin);
	cbit(motors_dir_port_reg,motors_RB_pin);
	sbit(motors_dir_port_reg,motors_LF_pin);
				
}

/**
 * @brief      Function to make Virtual Turtle to stop.
 */
void motors_stop(void) {
	
	// reset RB,RF,LF and LB
	
	cbit(motors_dir_port_reg,motors_RF_pin);
	cbit(motors_dir_port_reg,motors_LB_pin);
	cbit(motors_dir_port_reg,motors_RB_pin);
	cbit(motors_dir_port_reg,motors_LF_pin);
				
}