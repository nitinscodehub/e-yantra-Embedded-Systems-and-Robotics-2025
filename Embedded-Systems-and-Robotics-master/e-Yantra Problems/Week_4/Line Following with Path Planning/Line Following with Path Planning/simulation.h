/*
 * simulation.h
 *
 * Created: 22-03-2020 0:13:46
 * Author: Kotesh
 */ 


#ifndef SIMULATION_H_
#define SIMULATION_H_


#include <avr/io.h>				// Standard AVR IO Library
#include <avr/interrupt.h>		// Standard AVR Interrupt Library


// Definitions for ATmega328P micro-controller for sample projects in Proteus Demonstration
#if defined(__AVR_ATmega328P__)
	
	#define F_CPU								16000000

//---------------------------------- INPUT / OUTPUT PERIPHERALS -----------------------------------------------------


	// 3 White-Line sensors definitions
	
	#define		wl_sensors_left_ddr_reg					DDRB		///< Data Direction Register for PORTB
	#define		wl_sensors_left_port_reg				PORTB		///< Port Register for PORTB
	#define		wl_sensors_left_pin_reg					PINB		///< Pin Register for PORTB
	#define		left_wl_sensor_pin						3			///< PORTB Pin 3

	
	#define		wl_sensors_center_ddr_reg				DDRB		///< Data Direction Register for PORTB
	#define		wl_sensors_center_port_reg				PORTB		///< Port Register for PORTB
	#define		wl_sensors_center_pin_reg				PINB		///< Pin Register for PORTC
	#define		center_wl_sensor_pin					4			///< PORTB Pin 4
	
	
	#define		wl_sensors_right_ddr_reg				DDRC		///< Data Direction Register for PORTC
	#define		wl_sensors_right_port_reg				PORTC		///< Port Register for PORTB
	#define		wl_sensors_right_pin_reg				PINC		///< Pin Register for PORTC
	#define		right_wl_sensor_pin						0			///< PORTC Pin 0
	
	
	// Ultrasonic sensor definitions
	
	#define		ultrasonic_sensor_ddr_reg				DDRB		///< Data Direction Register for PORTC
	#define		ultrasonic_sensor_port_reg				PORTB		///< Port Register for PORTB
	#define		ultrasonic_sensor_pin_reg				PINB		///< Pin Register for PORTC
	#define		trigger_pin								0			///< PORTB Pin 0
	#define		echo_pin								1			///< PORTB Pin 1
	
	

	// Motor enable registers and pins
	
	#define		motors_pwm_ddr_reg						DDRD		///< Data Direction Register for PORTD
	#define		motors_pwm_port_reg						PORTD		///< Port Register for PORTD
	#define		motors_pwm_R_pin						PD5			///< PORTD Pin 5
	#define		motors_pwm_L_pin						PD3			///< PORTD Pin 3
	
	
	// Servo Pins
	
	#define		servo_ddr_reg							DDRB		///< Data Direction Register for PORTB
	#define		servo_port_reg							PORTB		///< Port Register for PORTB
	#define		servo_pin								PB2			///< PORTB Pin 2
	

	// Timer / Counter registers
	
	#define		OCR0BL_reg								OCR0B		///< Output Compare Register 0B
	#define		OCR0BH_reg								OCR0B		///< Output Compare Register 0B
	#define		TCCR0A_reg								TCCR0A		///< Timer / Counter Control Register 0A
	#define		TCCR0B_reg								TCCR0B		///< Timer / Counter Control Register 0B
	#define		TCNT0L_reg								TCNT0		///< Timer / Counter 5 Low Byte register
	
	
	#define		OCR2BL_reg								OCR2B		///< Output Compare Register 2B
	#define		OCR2BH_reg								OCR2B		///< Output Compare Register 2B
	#define		TCCR2A_reg								TCCR2A		///< Timer / Counter Control Register 2A
	#define		TCCR2B_reg								TCCR2B		///< Timer / Counter Control Register 2B
	#define		TCNT2L_reg								TCNT2		///< Timer / Counter 2 Low Byte register


	#define		OCR1BL_reg								OCR1B		// Output Compare Register 1B
	#define		OCR1BH_reg								OCR1B		// Output Compare Register 1B
	#define		TCCR1A_reg								TCCR1A		// Timer / Counter Control Register 1A
	#define		TCCR1B_reg								TCCR1B		// Timer / Counter Control Register 1B
	#define		TCNT1L_reg								TCNT1		// Timer / Counter 2 Low Byte register


	//Motor direction registers and pins
	
	#define		motors_dir_ddr_reg						DDRD		///< Data Direction Register for PORTD
	#define		motors_dir_port_reg						PORTD		///< Port Register for PORTD
	#define 	motors_RB_pin							PD7			///< PORTD Pin 7
	#define 	motors_RF_pin							PD6			///< PORTD Pin 6
	#define 	motors_LF_pin							PD2			///< PORTD Pin 2
	#define 	motors_LB_pin							PD4			///< PORTD Pin 4
	

#endif


#endif /* SIMULATION_H_ */