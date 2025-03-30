/*
 * sensors.h
 *
 * Created: 22-03-2020 1:00:43
 *  Author: Kotesh
 */ 


#ifndef SENSORS_H_
#define SENSORS_H_


// Makes three white line sensor pins as input and deactivates pull-up for these sensor pins
void wl_sensors_port_config();

// Makes Trigger pin as output and echo pin as input and activates pull-up for the echo pin
void ultrasonic_sensor_port_config();

// Activates trigger to detect the obstacle in the arena
void trigger();


#endif /* SENSORS_H_ */