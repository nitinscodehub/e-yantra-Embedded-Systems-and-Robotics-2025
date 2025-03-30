/*
 * simulation.h
 *
 * Created: 20-03-2020 22:20:36
 * Author: Kotesh
 */ 


#ifndef SIMULATION_H_
#define SIMULATION_H_


#include <avr/io.h>									// Standard AVR IO Library


// Definitions for ATmega328P micro-controller for sample projects in Proteus Demonstration
	
	#define F_CPU						16000000

	#define lcd_data_ddr_reg			DDRD
	#define lcd_control_ddr_reg			DDRB

	#define lcd_data_port_reg			PORTD
	#define lcd_control_port_reg		PORTB

	#define RS_pin						0			// PB0	( IO8 )
	#define RW_pin						2			// PB2	( IO10 )
	#define EN_pin						1			// PB1	( IO9 )

	#define DB7_pin						7			// PD7	( IO7 )
	#define DB6_pin						6			// PD6	( IO6 )
	#define DB5_pin						5			// PD5	( IO5 )
	#define DB4_pin						4			// PD4	( IO4 )




#endif /* SIMULATION_H_ */