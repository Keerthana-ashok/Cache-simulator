/***************************************************************************************************************************************
****************************************************************************************************************************************
 * Team ID:     B34
 * Author List:	Sanjay A C, Vikas H C, Keerthana A B, Sucharitha Bhat J S. 	  
 * Filename:	buzzer.h
 * Theme:		Search and Rescue
 * Functions:	buzzer_pin_config,buzzer_on,buzzer_on,buzzer					
 *						
 ***************************************************************************************************************************************
 ***************************************************************************************************************************************
 */

#ifndef BUZZER_H
#define BUZZER_H

/******************************************************************************************************************************
* Function Name: buzzer_pin_config
* Input:		 none
* Output:		 none
* Logic:		 configure ports to enable buzzer in robot
* Example call:	 buzzer_pin_config();
*******************************************************************************************************************************/

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

/******************************************************************************************************************************
* Function Name: buzzer_on
* Input:		 none
* Output:		 none
* Logic:		 turns ON the buzzer
* Example call:	 buzzer_on();
*******************************************************************************************************************************/

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

/******************************************************************************************************************************
* Function Name: buzzer_off
* Input:		 none
* Output:		 none
* Logic:		 turns OFF the buzzer
* Example call:	 buzzer_off();
*******************************************************************************************************************************/

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

/******************************************************************************************************************************
* Function Name: buzzer
* Input:		 sec ( number of seconds )
* Output:		 none
* Logic:		 turns ON the buzzer the buzzer for specified number of seconds
* Example call:	 buzzer_off();
*******************************************************************************************************************************/

void buzzer(int sec)	// To turn on buzzer for specified number of seconds
{
	buzzer_on();
	for(int i=0;i<sec;i++)
		_delay_ms(1000);
	buzzer_off();
}

#endif