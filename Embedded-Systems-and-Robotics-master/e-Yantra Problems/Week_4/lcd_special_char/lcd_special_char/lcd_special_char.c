/*! \mainpage Experiment: lcd_special_char
 *
 * @author     Kotesh
 * @date       2020/03/20
 *
 * \subsection Aim
 * To display special characters on LCD.
 *
 *
 * \subsection Connections
 * 
 *		RS_pin						:			PB0 				
 *		RW_pin						:			PB2 				
 *		EN_pin						:			PB1 					
 *
 *		DB7_pin						:			PD7 					
 *		DB6_pin						:			PD6 				
 *		DB5_pin						:			PD5 					
 *		DB4_pin						:			PD4 					
 *
 *
 * \subsection Macro_Definitions
 * Macros for LCD:				     <br>
 *
 *		lcd_data_ddr_reg		    :			DDRD 			
 *		lcd_control_ddr_reg			:			DDRB
 *		lcd_data_port_reg			:			PORTD 				
 *		lcd_control_port_reg		:			PORTB 				
 *
 *
 */ 


//---------------------------------- HEADER FILES -----------------------------------------------------

#include <avr/io.h>						// Header file includes the appropriate IO definitions for the device
#include "simulation.h"					// Header file included that contains macro definitions essential for simulation
#include <util/delay.h>					// Standard AVR Delay Library
#include "lcd.h"						// LCD Header file included that contains function definitions essential to deal with LCD


//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    After storing all custom characters in CGRAM, we can display it on LCD16x2.
 *			   So, it store all the custom characters and initializes the LCD, then
 * 			   build the custom characters at positions respectively. Then it prints a text 
 *			   message "Sine wave" for 5 seconds and then it prints the sine wave using 4 
 *			   Custom characters 1,2,3 and 4 for 5 seconds
 *
 *
 * @details	   Then it displays the text "Special characters" for 5 seconds and then it 
 *			   prints the special characters ! @ # $ % & * (in specified sequence)
 *			   for 5 seconds
 *
 *
 * @details	   Then it displays the text "Additional part" in the first row for 5 seconds 
 *
 *
 * @details	   Then it displays the text "Scrolling text" in the first row and then 
 *			   "with Custom char" in the second row for 5 seconds and the it scrolls display
 *			   left and right simultaneously with e-Yantra custom logo. It displays 
 *			   the text e-Yantra where 'e' is replaced with Custom character 5 
 *
 *
 * @details	   Then it displays the text "PAC-MAN" in the first row and the custom character 
 *			   of the PAC-MAN in the second row for 3 seconds and the it displays PAC-MAN
 *			   animation moving right and left simultaneously for once
 *
 *				
 * @details	   Then it displays the text "Running Sinewave" in the first row for 3 seconds 
 *			   then it shows the running sine wave for 5 seconds 
 *
 *
 * @return     0
 */

int main(void) {
	
	// Custom char set for LCD Module   
	
	unsigned char Character1[8] = {0x03,0x04,0x08,0x10,0x00,0x00,0x00,0x00};	// Positive sine wave part1
	unsigned char Character2[8] = {0x18,0x04,0x02,0x01,0x00,0x00,0x00,0x00};	// Positive sine wave part2
	unsigned char Character3[8] = {0x00,0x00,0x00,0x00,0x10,0x08,0x07,0x00};	// Negative sine wave part1
	unsigned char Character4[8] = {0x00,0x00,0x00,0x00,0x01,0x02,0x1C,0x00};	// Negative sine wave part2
	unsigned char Character5[8] = {0x1F,0x19,0x15,0x13,0x17,0x10,0x1F,0x00};	// e-Yantra Logo
	unsigned char Character6[8] = {0x0E,0x1B,0x1E,0x1C,0x1E,0x1F,0x0E,0x00};	// Pacman Right
	unsigned char Character7[8] = {0x0E,0x1B,0x0F,0x07,0x0F,0x1F,0x0E,0x00};	// Pacman Left
		
	

	lcd_port_config();					// Initialize the LCD port
	lcd_init();							// Initialize the LCD
	
	
	LCD_Custom_Char(1, Character1);		// Build Character1 at position 1 
	LCD_Custom_Char(2, Character2);		// Build Character2 at position 2 
	LCD_Custom_Char(3, Character3);		// Build Character3 at position 3 
	LCD_Custom_Char(4, Character4);		// Build Character4 at position 4 
	LCD_Custom_Char(5, Character5);		// Build Character4 at position 5
	LCD_Custom_Char(6, Character6);		// Build Character4 at position 6
	LCD_Custom_Char(7, Character7);		// Build Character4 at position 7


	lcd_clear();						// Clears LCD
	lcd_home();							// Sets the cursor's to home i.e. 1st row, 1st position
	lcd_wr_command(0x0C);				// Command for Display ON and Cursor OFF
	

	int count=0;						// Counter that counts the number of LCD bits printed
	
	
	// Displays the text "Sine wave" in the first row for 5 seconds
	
	lcd_string(1,4,"Sine wave");
		
	_delay_ms(5000);				
	
	lcd_clear();						// Clears LCD
	lcd_home();							// Sets the cursor's to home i.e. 1st row, 1st position
	

	count=0;							// Counter that counts the number of LCD bits printed	
	
	// Sine wave in the first row and second row for 5 seconds
	
	while(count<16){
		for(int i=0;i<4;i++){
			lcd_char(i+1);
			count++;
		}
	}
	
	count=0;							// Resetting the counter
	
	lcd_wr_command(0xC0);				// Sets the cursor to 2nd row 1st position
	
	
	while(count<16){
		for(int i=0;i<4;i++){
			lcd_char(i+1);
			count++;
		}
	}
	
	_delay_ms(5000);
	
	lcd_clear();						// Clears LCD

	// Displays the text "Special" in the first row and "Characters" in the second row for 5 seconds
	
	lcd_string(1,5,"Special");
	lcd_string(2,4,"Characters");
	
	_delay_ms(5000);
	
	
	lcd_clear();						// Clears the display
	lcd_cursor(1,5);					// Sets the cursor to its appropriate position
	
	
	lcd_char(0x21);						// Character code for '!'
	lcd_char(0x40);						// Character code for '@'
	lcd_char(0x23);						// Character code for '#'
	lcd_char(0x24);						// Character code for '$'
	lcd_char(0x25);						// Character code for '%'
	lcd_char(0x26);						// Character code for '&'
	lcd_char(0x2A);						// Character code for '*'
	
	
	_delay_ms(5000);
	
	lcd_clear();						// Clears LCD
	
	// Displays the text "Additional part" in the first row for 5 seconds
	
	lcd_string(1,2,"Additional part");
	
	_delay_ms(5000);
	
	
	lcd_clear();						// Clears LCD

	//displays the text "Scrolling text" in the first row and then
	//"with Custom char" in the second row for 5 seconds and the it scrolls display
	//left and right simultaneously with e-Yantra custom logo. It displays
	//the text e-Yantra where 'e' is replaced with Custom character 5
	
	lcd_string(1,2,"Scrolling text");
	lcd_string(2,1,"with Custom Char");
	
	_delay_ms(5000);
	
	lcd_clear();						// Clears LCD
	
	lcd_wr_char(1,1,5);					// 5 is the Custom character for e-Yantra logo
	lcd_string(1,2,"-Yantra");
	_delay_ms(250);
	
	for(int x=0;x<2;x++)
	{
		for(int i=0;i<8;i++)
		{
			lcd_wr_command(0x1c);		// LCD command for right shift
			_delay_ms(250);
		}
		
		_delay_ms(100);
		
		for(int i=0;i<8;i++)
		{
			lcd_wr_command(0x18);		// LCD command for left shift
			_delay_ms(250);			
		}

		_delay_ms(1000);
		
	}
	
	lcd_clear();						// Clears LCD
	
	//displays the text "PAC-MAN" in the first row and the custom characters
	//of the PAC-MAN(both left and right) in the second row for 3 seconds and the it displays PAC-MAN
	//animation moving right and left simultaneously for once
	
	lcd_string(1,6,"PAC-MAN");
	lcd_wr_char(2,8,7);					// 7 is the Custom character for PAC-MAN left facing
	lcd_wr_char(2,9,6);					// 5 is the Custom character for PAC-MAN right facing
	
	_delay_ms(3000);
	
	lcd_clear();						// Clears LCD
	
	char space=' ';
	char text[]="---------------";
	lcd_clear();
		
	lcd_wr_char(1,1,6);
	lcd_string(1,2,text);
	for(int i=2;i<17;i++)
	{
		for(int x=1;x<i;x++)
		{
			lcd_wr_char(1,x,space);
		}
		lcd_wr_char(1,i,6);
		_delay_ms(250);
			
	}
		
	_delay_ms(500);
	
	lcd_clear();						// Clears LCD
	
	lcd_wr_char(2,16,7);
	lcd_string(2,1,text);
	for(int i=15;i>0;i--)
	{
		for(int x=16;x>i;x--)
		{
			lcd_wr_char(2,x,space);
		}
		lcd_wr_char(2,i,7);
		_delay_ms(250);
		
	}
	
	_delay_ms(500);
		
	lcd_clear();						// Clears LCD
	
	// displays the text "Running Sinewave" in the first row for 3 seconds
	// then it shows the running sine wave for 5 seconds
	
	lcd_string(1,1,"Running Sinewave");
	
	_delay_ms(3000);
	
	lcd_clear();						// Clears LCD

	for(int i=0;i<5;i++)
	{
		lcd_cursor(1,1);
		for(int i=0;i<4;i++)
		{
			lcd_char(1);
			lcd_char(2);
			lcd_char(3);
			lcd_char(4);
		}
		
		lcd_cursor(1,1);
		for(int i=0;i<4;i++)
		{
			lcd_char(2);
			lcd_char(3);
			lcd_char(4);
			lcd_char(1);
		}
		
		lcd_cursor(1,1);
		for(int i=0;i<4;i++)
		{
			lcd_char(3);
			lcd_char(4);
			lcd_char(1);
			lcd_char(2);
		}
		
		lcd_cursor(1,1);
		for(int i=0;i<4;i++)
		{
			lcd_char(4);
			lcd_char(1);
			lcd_char(2);
			lcd_char(3);

		}
		
	}
	
	_delay_ms(200);
	
	lcd_clear();						// Clears LCD
	
	lcd_string(1,4,"Thank you");
	lcd_string(2,2,"Team e-Yantra");
	
	
	return 0;
}
//---------------------------------- END ------------------------------------------------------------------

