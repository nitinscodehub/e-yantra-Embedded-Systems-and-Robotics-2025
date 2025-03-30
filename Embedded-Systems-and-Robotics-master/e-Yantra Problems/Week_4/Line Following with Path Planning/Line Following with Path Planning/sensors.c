/*
 * sensors.c
 *
 * Created: 22-03-2020 1:01:54
 *  Author: Kotesh
 */ 

#include "simulation.h"				// Header file included that contains macro definitions essential for Virtual turtle
#include <stdbool.h>				// Standard C Library for Boolean Type
#include <util/delay.h>				// Standard AVR Delay Library
#include "sensors.h"


#define sbit( reg, bit )	reg |= ( 1 << bit )			// Macro defined for Setting a bit of any register
#define cbit( reg, bit )	reg &= ~( 1 << bit )		// Macro defined for Clearing a bit of any register


/**
 * @brief      Makes three white line sensor pins as input and deactivates pull-up for these sensor pins
 */
void wl_sensors_port_config(){
	
	// Make three white line sensor pins as input
	
	cbit(wl_sensors_left_ddr_reg,left_wl_sensor_pin);
	cbit(wl_sensors_center_ddr_reg,center_wl_sensor_pin);
	cbit(wl_sensors_right_ddr_reg,right_wl_sensor_pin);
	
	// Deactivate pull-up for for three white line sensor pins
	
	cbit(wl_sensors_left_port_reg,left_wl_sensor_pin);
	cbit(wl_sensors_center_port_reg,center_wl_sensor_pin);
	cbit(wl_sensors_right_port_reg,right_wl_sensor_pin);

}


/**
 * @brief      Makes Trigger pin as output and echo pin as input and activates pull-up for the echo pin
 */
void ultrasonic_sensor_port_config(){
	
	ultrasonic_sensor_ddr_reg |= (1 << trigger_pin);
	ultrasonic_sensor_ddr_reg &= ~(1 << echo_pin);
	ultrasonic_sensor_port_reg |= (1 << echo_pin);

}

/**
 * @brief      Activates trigger to detect the obstacle
 */
void trigger(){
	
	ultrasonic_sensor_port_reg |= (1 << trigger_pin);
	_delay_us(15);
	
	ultrasonic_sensor_port_reg &= ~( 1 << trigger_pin);

}





