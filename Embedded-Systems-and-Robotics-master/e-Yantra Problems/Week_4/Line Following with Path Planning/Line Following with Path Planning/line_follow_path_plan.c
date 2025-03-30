/*! \mainpage Experiment: line_follow_path_plan
 *
 * @author     Kotesh
 * @date       2020/03/22
 *
 * \subsection Aim
 * Line Following with Path Planning
 *
 * \subsection Connections
 * Left white line sensor						:	PB3 				
 * Center white line sensor						:	PB4					
 * Right white line sensor 						:	PC0					
 * Right motor PWM								:	PD5					
 * Left motor PWM								:	PD3					
 * Motor Right_Backward							:	PD7					
 * Motor Right_Forward							:	PD6					
 * Motor Left_Backward							:	PD4					
 * Motor Left_Forward							:	PD2		
 * Trigger pin									:	PB0
 * Echo pin										:	PB1
 * Servo pin									:   PB2
 *
 *
 *			
 *
 * \subsection Macro_Definitions
 * Macros for 3 White-Line sensors definitions:				<br>
 *		wl_sensors_left_ddr_reg					:	DDRB 						
 *		wl_sensors_left_port_reg				:	PORTB 						
 *		wl_sensors_left_pin_reg					:	PINB 						
 *		left_wl_sensor_pin						:	3	 						
 *		wl_sensors_center_ddr_reg				:	DDRB 						
 *		wl_sensors_center_port_reg				:	PORTB 						
 *		wl_sensors_center_pin_reg				:	PINB 						
 *		center_wl_sensor_pin					:	4 								
 *		wl_sensors_right_ddr_reg				:	DDRC 						
 *		wl_sensors_right_port_reg				:	PORTC 						
 *		wl_sensors_right_pin_reg				:	PINC 						
 *		right_wl_sensor_pin						:	0 								
 *
 * Macros for Ultrasonic sensor definitions:				<br>
 *		ultrasonic_sensor_ddr_reg				:	DDRB		
 *		ultrasonic_sensor_port_reg				:	PORTB		
 *		ultrasonic_sensor_pin_reg				:	PINB		
 *		trigger_pin								:	0			
 *		echo_pin								:	1			
 *
 * Macros for Servo motor registers and pins:				<br>
 *			servo_ddr_reg						:	DDRB		
 *			servo_port_reg						:	PORTB		
 *			servo_pin							:	PB2			
 * 
 * Macros for  Motor enable registers and pins:				<br>
 *		motors_pwm_ddr_reg						:	DDRD 						
 *		motors_pwm_port_reg						:	PORTD 						
 *		motors_pwm_R_pin						:	PD5 							
 *		motors_pwm_L_pin						:	PD3 						
 * 
 * Macros for Motor direction registers and pins:				<br>
 *		motors_dir_ddr_reg						:	DDRD 				
 *		motors_dir_port_reg						:	PORTD 				
 *		motors_RB_pin							:	PD7 				
 *		motors_RF_pin							:	PD6 				
 *		motors_LF_pin							:	PD2 				
 *		motors_LB_pin							:	PD4 				
 * 
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include <avr/io.h>			// Header file includes the appropriate IO definitions for the device  
#include "simulation.h"		// Header file included that contains macro definitions essential for simulation
#include <util/delay.h>		// Standard AVR Delay Library
#include "motor.h"			// Header file included that contains the functions that are essential for motors  
#include "sensors.h"		// Header file included that contains the functions that are essential for sensors
#include <avr/interrupt.h>	// Header file included that contains the functions that are essential for interrupts


//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 *
 *
 *		       First Initializes the white-line sensors present on the virtual turtle and the motor pins and its
 *			   PWM pins and initializes TIMER 0 and TIMER 2 of Channel B for Pulse Width Modulation(PWM)
 * 			   and the white line sensors used in the virtual turtle are Digital sensors. So ADC conversion is not
 *			   required here and we can read the data directly from its respective pins. Then we initialize the required
 *			   variables and then Path is initialized that the virtual turtle is going to follow.It considers a 
 *			   while loop for a each sub-path. Suppose if we consider the Path is {4, 5, 7, 8, 6},then the
 *			   path and its loop for its sub-part will be:
 *	 
 *					
 * 
 *
 *			 							   STOP
 *			 								|
 *	  		 		________________________|_______________________
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |______|______|______|______|______|______|______|
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |______|______|______|______|______|______|______|
 *			 	  |      |      |      |      |      |      |      |
 *			 	   |      |      |      |      |      |      |      |
 *			 	  |      |      |      |      |      |      |      |
 *			 	   |______|______|______|______|______|______|______|
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |      |      |      |      |      |      |      |
 *			 	   |______|______|______|______|______|______|______|
 *			 	   |      |      |      |      |      |      |      |
 *			 	    \      \      \      \      \      \      \      \
 *			 	    /      /      /      /      /      /      /      /
 *			 	   |______|______|______|______|______|______|______|
 *			 	   1	  2      3      4   |  5      6      7      8
 *			 								|
 *			 							  START
 *
 *
 *
 *
 *
 *
 *
 *
 *									   STOP
 *			        LOOP 12 --------->  *	  LOOP 11
 *										*	  ?	
 *										***********
 *												  *           
 *							   LOOP 10 ---------> *     LOOP 9   
 *												  *      ?   
 *												  ***************
 *			        											*
 *														LOOP 7	*  <----------LOOP 8
 *															??   *
 *														 ********
 *														 *     
 *											   LOOP 5	 *   <----------LOOP 6  
 *			                                      ??      *      
 *										   ***************
 *			                               *    
 *							       LOOP 3  *  <----------LOOP 4    
 *									   ??   *      
 *									********
 *									*     
 *		       LOOP 2 --------->    *      
 *									*      
 *									*****
 *									4	*	<----------LOOP 1
 *										*
 *									  START
 *
 *
 *
 *
 *
 *
 *
 *					
 *						For the motion of the virtual turtle, It detects the black line using the white line sensors and 
 *			accordingly the Robot moves.As we used the digital white line sensors, We can directly read the data from the 
 *			sensors. If Center white line Sensor return high then the virtual turtle move forward i.e both the RF and LF will 
 *			be set HIGH with some speed using PWM. Similarly if Left white line or both both Left and Center white line sensors
 *			return high, then the virtual turtle move move towards right(Soft Right) i.e only LF will be set HIGH. Similarly 
 *			if Right white line or both Right and Center white line sensor return high then the virtual bot turns towards 
 *			left(Soft Left) i.e only RF will be set HIGH. If all three white line sensors return high, the virtual turtle 
 *			stops for a second and takes the next move according to the path. If all the three sensors return zero, the turtle 
 *			never moves until any of the white line sensor return High.
 *
 *
 *					If we consider given path, Initially it will be at (1636,2500) with an angle 0 degrees i.e the start point 
 *			of the arena. Initially it is on the center white line sensor. So initially it moves forward. Now lets see how each loop
 *			is going to work.
 *		
 *		Loop-1 :
 *				Initially the turtle moves forward. It moves forward until it all the three sensors return High i.e until 
 *				it meets the junction. When it meets the junction, The turtle stops for a second and then it moves right 
 *				or left according to the value of Path[0]. If Path[0] > 5 , then the turtle moves back with some delay and
 *				turns left. If Path[0] >= 5 , the turtle moves back with some delay and turns right. After taking the turn 
 *				turtle moves forward until it meets the point Path[0]. While reaching Path[0], Turtle stops for a second then
 *				it moves forward until Path[0]. After reaching Path[0], the loop breaks.
 *
 *		Loop-2 :
 *				At Path[0], if Path[0] < 5 ,the the turtle moves back and turn right and moves forward until it meets a junction
 *				If Path[0] >=5 , then the turtle moves back and turns left and continues to move forward until it meets a junction
 *				After meeting the junction, the turtle stops and the loop breaks
 *
 *				Similarly all the other loops are executed. The turtle moving in angle 0 have a loop with even number
 *			(Loop 2,4,6,8,10,12). The turtle taking left or right turns and moving forward are considered in odd number loop
 *			(Loop 1,3,5,7,9,11). If there is no need to take a left or right turn excluding at START and STOP positions, then there 
 *			is no need to run the odd numbered loops. So initially if Path[0]==Path[1] then loop 3 breaks without executing.
 *		
 *
 *      
 * @return     0
 */


int main(void)
{
	
	wl_sensors_port_config();			// Initializes the three white line sensors
	ultrasonic_sensor_port_config();	// Initializes the Ultrasonic sensor
	servo_pin_config();					// Initializes the servo motor connected to ultrasonic sensor
	pwm_pin_config();					// Initializes pwm pins as output
	timer_pwm_init();					// Initializes Timer 0 and Timer 2
	motors_pin_config();				// Initializes the motor pins
   

	int flag1=0;						// set the flag so that further white line sensor comparison is enabled or disabled
	int count1=0;						// count1 is set when takes a left or right turn when it is at an angle 90 or 180
	int count2=0;						// count2 is set when takes a left or right turn when it is at an angle 0
	int junction=0;						// counts the number of junctions the turtle crossed


	int Path[5] = {4, 5, 7, 8, 6};		// Path that is going to be followed by Virtual Turtle
	   
	/*
	Path_1 : {4, 5, 7, 8, 6}
		
	Path_2 = {2, 3, 6, 2, 4}
	
	*/


	// Variables for storing the white line sensor values
	
	unsigned char left_wl_sensor_data,center_wl_sensor_data,right_wl_sensor_data;	

	
	
	//---------------------------------------------------------------------------------------------------------------------
	
	
	// LOOP 1

    /* Initially the turtle moves forward. It moves forward until it all the three sensors return High i.e until
       it meets the junction. When it meets the junction, The turtle stops for a second and then it moves right
       or left according to the value of Path[0]. If Path[0] > 5 , then the turtle moves back with some delay and
       turns left. If Path[0] >= 5 , the turtle moves back with some delay and turns right. After taking the turn
       turtle moves forward until it meets the point Path[0]. While reaching Path[0], Turtle stops for a second then
       it moves forward until Path[0]. After reaching Path[0], the loop breaks.
	
	*/ 
	

    while(1)
    {
		
		flag1=0;
	
		left_wl_sensor_data  = wl_sensors_left_pin_reg & (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg & (1 << right_wl_sensor_pin);
		
		
		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{
			flag1=1;
			motors_stop();
			_delay_ms(1000);
			junction++;
		
		
			if(((junction==6-Path[0]) && (Path[0]<5)) || ((junction==Path[0]-3) && (Path[0]>=5)))
			{
				motors_stop();
				break;
			}
		
			if(Path[0]<5 && count1==0)
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(710);
			
				motors_move_left();
				speed(0,208);
				_delay_ms(1000);
			
				motors_move_forward();
				speed(100,100);
				_delay_ms(175);
				
				count1=1;
				
			}

			if(Path[0]>= 5 && count1==0)
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(710);
				
				motors_move_right();
				speed(211,0);
				_delay_ms(1005);
				
				motors_move_forward();
				speed(100,100);
				_delay_ms(175);

				count1=1;
				
			}


			
			if( (Path[0]<5) && (count1>0))
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
				count1=2;
				
			}

			if( (Path[0] >= 5) && (count1>0))
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
				count1=2;
				
			}
				
		}
		
		if(center_wl_sensor_data && !right_wl_sensor_data && !left_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(100,100);
		}
		
		if((left_wl_sensor_data || (center_wl_sensor_data && left_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(15,60);
		}
		
		if((right_wl_sensor_data || (center_wl_sensor_data && right_wl_sensor_data)) && flag1==0)
		{	
			flag1=1;
			motors_move_forward();
			speed(60,15);
		}
	
		
	_delay_ms(250);

	}
	
	//---------------------------------------------------------------------------------------------------------------------

	// LOOP 2
	
	/*
		At Path[0], if Path[0] < 5 ,the the turtle moves back and turn right and moves forward until it meets a junction
		If Path[0] >=5 , then the turtle moves back and turns left and continues to move forward until it meets a junction
		After meeting the junction, the turtle stops and the loop breaks
		
	*/

	while(1)
	{
		flag1=0;

		if(count2==0)
		{
			if(Path[0] >= 5)
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(534);

				motors_move_left();
				speed(0,208);
				_delay_ms(1000);
				count2=1;

			}

			if(Path[0]<5)
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(532);

				motors_move_right();
				speed(215,0);
				_delay_ms(1000);
				count2=1;

			}

		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg &   (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg &  (1 << right_wl_sensor_pin);

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{

			motors_stop();
			_delay_ms(1000);
			break;
			
		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(50,50);
		}
		 
		if( ( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data ) ) && flag1==0)
		{
			flag1=1;
			motors_move_left();
			speed(31,50);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_right();
			speed(50,31);
		}
		

	}

	//---------------------------------------------------------------------------------------------------------------------
	

	// LOOP 3
	
	/*
		If Path[0] > Path[1] , then the turtle moves back with some delay and turns left. If Path[0] < Path[1], the
		turtle moves back with some delay and turns right. After taking the turn turtle moves forward until it meets 
		the point Path[1]. While reaching Path[1], Turtle stops for a second then it moves forward until Path[1]. 
		After reaching Path[1], the loop breaks.If there is no need to take a left or right turn excluding at 
		START and STOP positions, then there is no need to run the odd numbered loops. So initially if 
		Path[0]==Path[1] then loop 3 breaks without executing.
	
	*/
	
	
	count1=0;					// Resetting count1
	junction=0;					// Resetting junction

	while(1)
	{

		flag1=0;
		
		if(Path[0] == Path[1])
			break;
		

		if(Path[1]<Path[0] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(658);
			
			motors_move_left();
			speed(0,208);
			_delay_ms(1000);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);
			
			count1=1;
			
		}

		if(Path[1] > Path[0] && count1==0) 
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(345);
			
			motors_move_right();
			speed(211,0);
			_delay_ms(953);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);

			count1=1;
			
		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg & (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg & (1 << right_wl_sensor_pin);
		

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{

			flag1=1;
			motors_stop();
			_delay_ms(1000);
			junction++;
	
			if (((junction==Path[0]-Path[1]) && (Path[1]<Path[0])) || ((junction==Path[1]-Path[0]) && (Path[1]>=Path[0])))
			{
				motors_stop();
				break;
			}

			
			if( (Path[1]<Path[0]) && (count1>0))
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
				count1=1;
				
			}

			if( (Path[0] >= Path[0]) && (count1>0))
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
				count1=1;
				
			}
			
		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(100,100);
		}
		
		if(( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data )) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(15,60);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(60,15);
		}
		
		
		_delay_ms(250);
	}

	//---------------------------------------------------------------------------------------------------------------------
	
	
	// LOOP 4
	
	/*
		At Path[0], if Path[0] >= Path[1] ,the the turtle moves back and turn right and moves forward until it meets a junction
		If Path[0] < Path[1] , then the turtle moves back and turns left and continues to move forward until it meets a junction
		After meeting the junction, the turtle stops and the loop breaks. If Path[0]==Path[1], then robot will never turn 
		right or left in this loop.
	
	*/
	
	
	count2=0;		// Resetting count2

	while(1)		
	{
		flag1=0;

		if(count2==0 && (Path[0] != Path[1]))
		{
			if(Path[0]<Path[1])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(550);

				motors_move_left();
				speed(0,208);
				_delay_ms(1008);
				count2=1;

			}

			if(Path[0] >= Path[1])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(532);

				motors_move_right();
				speed(215,0);
				_delay_ms(1000);
				count2=1;

			}

		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg &   (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg &  (1 << right_wl_sensor_pin);

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{

			motors_stop();
			_delay_ms(1000);
			break;
			
		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(50,50);
		}
		
		if( ( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data ) ) && flag1==0)
		{
			flag1=1;
			motors_move_left();
			speed(31,50);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_right();
			speed(50,31);
		}
		
	
	}

	//---------------------------------------------------------------------------------------------------------------------


	// LOOP 5
	
	/*
		If Path[1] > Path[2] , then the turtle moves back with some delay and turns left. If Path[1] < Path[2],
		the turtle moves back with some delay and turns right. After taking the turn turtle moves forward until
		it meets the point Path[2]. While reaching Path[2], Turtle stops for a second then it moves forward until 
		Path[2]. After reaching Path[2], the loop breaks.If there is no need to take a left or right turn excluding at
		START and STOP positions, then there is no need to run the odd numbered loops. So initially if
		Path[1]==Path[2] then loop 3 breaks without executing.
	
	*/
	
	
	count1=0;			// Resetting count1
	junction=0;			// Resetting junction


	while(1)	
	{
		
		flag1=0;
		

		if(Path[2]<Path[1] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(658);
			
			motors_move_left();
			speed(0,208);
			_delay_ms(1000);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);
			
			count1=1;
			
		}

		if(Path[2] > Path[1] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(348);
			
			motors_move_right();
			speed(211,0);
			_delay_ms(990);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);

			count1=1;
			
		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg & (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg & (1 << right_wl_sensor_pin);
		

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{

			flag1=1;
			motors_stop();
			_delay_ms(1000);
			
			junction++;
			
			if (((junction==Path[1]-Path[2]) && (Path[2]<Path[1])) || ((junction==Path[2]-Path[1]) && (Path[2]>=Path[1])))
			{
				motors_stop();
				break;
			}

			
			if(count1>0)
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
				count1=1;
				
			}
			
			
			
		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(100,100);
		}
		
		if(( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data )) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(15,60);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(60,15);
		}
		
		
		_delay_ms(250);

	}
	
	//---------------------------------------------------------------------------------------------------------------------

	
	// LOOP 6
	
	/*
		At Path[2], if Path[1] > Path[2] ,the the turtle moves back and turn right and moves forward irrespective with 
		white line sensors until it meets a junction. Similarly if Path[1] < Path[2] , then the turtle moves back and 
		turns left and continues to move forward until it meets a junction. After meeting the junction, the turtle stops
		and the loop breaks. If Path[1]==Path[2], then robot will never turn right or left in this loop.
	
	
	*/
	
	
	count2=0;		// Resetting count2

	while(1)		
	{

		if(count2==0 && (Path[1] != Path[2]))
		{
			if(Path[1] < Path[2])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(450);

				motors_move_left();
				speed(0,208);
				_delay_ms(1020);
				count2=1;

			}

			if(Path[1] > Path[2])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(532);

				motors_move_right();
				speed(215,0);
				_delay_ms(1000);
				count2=1;

			}

		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg &   (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg &  (1 << right_wl_sensor_pin);

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data)
		{

			motors_stop();
			_delay_ms(1000);
			break;
			
		}
		
		else
		{
			motors_move_forward();
			speed(50,50);
		}
		_delay_ms(250);
		
	}

	//---------------------------------------------------------------------------------------------------------------------
	
	
	// LOOP 7
	
	/*
		If Path[2] > Path[3] , then the turtle moves back with some delay and turns left. If Path[2] < Path[3],
		the turtle moves back with some delay and turns right. After taking the turn turtle moves forward until
		it meets the point Path[3]. While reaching Path[3], Turtle stops for a second then it moves forward until
		Path[3]. After reaching Path[3], the loop breaks.If there is no need to take a left or right turn excluding at
		START and STOP positions, then there is no need to run the odd numbered loops. So initially if
		Path[2]==Path[3] then loop 3 breaks without executing.
	
	*/
	
	count1=0;			// Resetting count1
	junction=0;			// Resetting junction


	while(1)
	{
		flag1=0;
		

		if(Path[3]<Path[2] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(658);
			
			motors_move_left();
			speed(0,208);
			_delay_ms(1000);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);
			
			count1=1;
			
		}

		if(Path[3] > Path[2] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(475);
			
			motors_move_right();
			speed(211,0);
			_delay_ms(1015);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);

			count1=1;
			
		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg & (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg & (1 << right_wl_sensor_pin);
		

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{

			flag1=1;
			motors_stop();
			_delay_ms(1000);
			junction++;
			
			
			if (((junction==Path[2]-Path[3]) && (Path[3]<Path[2])) || ((junction==Path[3]-Path[2]) && (Path[3]>=Path[2])))
			{
				motors_stop();
				break;
			}

			
			if((count1>0))
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
				count1=1;
				
			}
		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(100,100);
		}
		
		if(( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data )) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(15,60);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(60,15);
		}
		
		
		_delay_ms(250);

	}
	
	
	//---------------------------------------------------------------------------------------------------------------------


	// LOOP 8
	
	/*
		At Path[3], if Path[2] > Path[3] ,the the turtle moves back and turn right and moves forward irrespective with
		white line sensors until it meets a junction. Similarly if Path[2] < Path[3] , then the turtle moves back and
		turns left and continues to move forward until it meets a junction. After meeting the junction, the turtle stops
		and the loop breaks. If Path[2]==Path[3], then robot will never turn right or left in this loop.
		
	
	*/
	
	
	count2=0;		// Resetting count2

	while(1)		
	{
		flag1=0;

		if(count2==0 && (Path[2] != Path[3]))
		{
			if(Path[2] < Path[3])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(650);

				motors_move_left();
				speed(0,206);
				_delay_ms(1015);
				count2=1;

			}

			if(Path[2] > Path[3])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(532);

				motors_move_right();
				speed(215,0);
				_delay_ms(990);
				count2=1;

			}

		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg &   (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg &  (1 << right_wl_sensor_pin);

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && flag1==0)
		{

			motors_stop();
			_delay_ms(1000);
			break;
			
		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(50,50);
		}
		
		if( ( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data ) ) && flag1==0)
		{
			flag1=1;
			motors_move_left();
			speed(31,50);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_right();
			speed(50,31);
		}
		
		
		_delay_ms(250);
		
	}
	
	
	//---------------------------------------------------------------------------------------------------------------------


	// LOOP 9
	
	/*
		If Path[3] > Path[4] , then the turtle moves back with some delay and turns left. If Path[3] < Path[4],
		the turtle moves back with some delay and turns right. After taking the turn turtle moves forward until
		it meets the point Path[4]. While reaching Path[4], Turtle stops for a second then it moves forward until
		Path[4]. After reaching Path[4], the loop breaks. If there is no need to take a left or right turn excluding at
		START and STOP positions, then there is no need to run the odd numbered loops. So initially if
		Path[3]==Path[4] then loop 3 breaks without executing.
	
	
	*/
	
	
	count1=0;		// Resetting count1
	junction=0;		// Resetting junction

	while(1)	
	{
		flag1=0;
		

		if(Path[4]<Path[3] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(500);
			
			motors_move_left();
			speed(0,208);
			_delay_ms(1015);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);
			
		
			count1=1;
			
		}

		if(Path[4] > Path[3] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(475);
			
			motors_move_right();
			speed(211,0);
			_delay_ms(988);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);

		
			count1=1;
			
		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg & (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg & (1 << right_wl_sensor_pin);
		

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{

			flag1=1;
			motors_stop();
			_delay_ms(1000);
			junction++;
			
			if (((junction==Path[3]-Path[4]) && (Path[4]<Path[3])) || ((junction==Path[4]-Path[3]) && (Path[4]>=Path[3])))
			{
				motors_stop();
				break;
			}

			
			if( (Path[4]<Path[3]) && (count1>0))
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
	
				count1=1;
				
			}

			if( (Path[4] > Path[3]) && (count1>0))
			{
				motors_move_forward();
				speed(110,110);
				_delay_ms(165);
			
				count1=1;
				
			}
			
			
			
		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(100,100);
		}
		
		if(( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data )) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(15,60);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(60,15);
		}
		
		_delay_ms(250);

	}
	
	//---------------------------------------------------------------------------------------------------------------------


	// LOOP 10
	
	/*
		At Path[4], if Path[3] > Path[4] ,the the turtle moves back and turn right and moves forward irrespective with
		white line sensors until it meets a junction. Similarly if Path[3] < Path[4] , then the turtle moves back and
		turns left and continues to move forward until it meets a junction. After meeting the junction, the turtle stops
		and the loop breaks. If Path[3]==Path[4], then robot will never turn right or left in this loop.
	
	*/
	
	
	count2=0;		// Resetting count2

	while(1)		
	{

		if(count2==0 && (Path[3] != Path[4]))
		{
			if(Path[3] < Path[4])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(590);

				motors_move_left();
				speed(0,208);
				_delay_ms(1000);
				count2=1;

			}

			if(Path[3] > Path[4])
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(525);

				motors_move_right();
				speed(215,0);
				_delay_ms(990);
				count2=1;

			}

		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg &   (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg &  (1 << right_wl_sensor_pin);

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data)
		{

			motors_stop();
			_delay_ms(1000);
			break;
			
		}
		
		else
		{
			motors_move_forward();
			speed(50,50);
		}
		_delay_ms(250);
		
	}
	
	//---------------------------------------------------------------------------------------------------------------------
	
	
	// LOOP 11
	
	/*
		If Path[3] > Path[4] , then the turtle moves back with some delay and turns left. If Path[3] < Path[4],
		the turtle moves back with some delay and turns right. After taking the turn turtle moves forward until
		it meets the end point. While reaching end point, Turtle stops for a second then it moves forward until
		end point. After reaching end point, the loop breaks.

	*/
	
	
	count1=0;			// Resetting count1
	junction=0;			// Resetting count1


	while(1)	// Loop 11
	{
		flag1=0;
		

		if(Path[4] < Path[3] && count1==0) 
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(500);
			
			motors_move_left();
			speed(0,208);
			_delay_ms(1010);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);
			
			count1=1;
			
		}

		if(Path[4] > Path[3] && count1==0)
		{
			motors_move_backward();
			speed(100,100);
			_delay_ms(475);
			
			motors_move_right();
			speed(211,0);
			_delay_ms(1010);
			
			motors_move_forward();
			speed(100,100);
			_delay_ms(175);

			count1=1;
			
		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg & (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg & (1 << right_wl_sensor_pin);
		

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data && (flag1==0))
		{

			flag1=1;
			motors_stop();
			_delay_ms(1000);
			junction++;
			
			if (((junction==5-Path[4]) && (Path[4]<5)) || ((junction==Path[4]-4) && (Path[4]>=5)))
			{
				motors_stop();
				break;
			}


			motors_move_forward();
			speed(110,110);
			_delay_ms(165);
				

		}
		
		if(center_wl_sensor_data && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(100,100);
		}
		
		if(( left_wl_sensor_data ||  ( left_wl_sensor_data && center_wl_sensor_data )) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(15,60);
		}
		
		if(( right_wl_sensor_data || ( right_wl_sensor_data && center_wl_sensor_data)) && flag1==0)
		{
			flag1=1;
			motors_move_forward();
			speed(60,15);
		}
		
		_delay_ms(250);

	}
	
	//---------------------------------------------------------------------------------------------------------------------
	
	
	// LOOP 12
	
	/*
		At the end point, if Path[3] > Path[4] ,the the turtle moves back and turn right and moves forward irrespective with
		white line sensors until it meets a junction. Similarly if Path[3] < Path[4] , then the turtle moves back and
		turns left and continues to move forward until it meets the STOP point. After meeting the junction, the turtle stops
		and the loop breaks.

	*/
	
	count2=0;		// Resetting count2

	while(1)		
	{

		if(count2==0)
		{
			if(Path[4]<5)
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(725);

				motors_move_left();
				speed(0,208);
				_delay_ms(1000);
				count2=1;

			}

			if(Path[4]>=5)
			{
				motors_move_backward();
				speed(100,100);
				_delay_ms(475);

				motors_move_right();
				speed(215,0);
				_delay_ms(970);
				count2=1;

			}

		}
		
		left_wl_sensor_data  = wl_sensors_left_pin_reg &   (1 << left_wl_sensor_pin);
		center_wl_sensor_data= wl_sensors_center_pin_reg & (1 << center_wl_sensor_pin);
		right_wl_sensor_data = wl_sensors_right_pin_reg &  (1 << right_wl_sensor_pin);

		if(center_wl_sensor_data && left_wl_sensor_data && right_wl_sensor_data)
		{

			motors_stop();
			_delay_ms(1000);
			break;
			
		}
		
		else
		{
			motors_move_forward();
			speed(50,50);
		}
		_delay_ms(250);
		
	}
	

	return 0;
}



//---------------------------------------------------------------------------------------------------------------------
