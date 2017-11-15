/***************************************************************************************************************************************
****************************************************************************************************************************************
 * Team ID:     eYRCPlus-SR#935
 * Author List:	Sanjay A C, Vikas H C, Keerthana A B, Sucharitha Bhat J S. 	  
 * Filename:	servo.h
 * Theme:		Search and Rescue
 * Functions:	servo1_pin_config,servo_init,timer1_init,servo_1,servo_1_free,servo,first_aid_kit_deposit
 * Global Variables: 	c_degrees,			(To track the current position of servo),
						first_aid_kit_cnt	(To track number of first aid kit deposited),
 ***************************************************************************************************************************************
 ***************************************************************************************************************************************
 */
#ifndef SERVO_H_
#define SERVO_H_

int c_degrees=0;
int first_aid_kit_cnt;

/******************************************************************************************************************************
* Function Name: servo1_pin_config
* Input:		 none
* Output:		 none
* Logic:		 configure ports connected to servo1
* Example call:	 servo1_pin_config();
*******************************************************************************************************************************/
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/******************************************************************************************************************************
* Function Name: servo_init
* Input:		 none
* Output:		 none
* Logic:		 Initialse servo pin
* Example call:	 servo_init();
*******************************************************************************************************************************/
void servo_init(void)
{
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	timer1_init();
	servo_1(45);
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
/******************************************************************************************************************************
* Function Name: timer1_init
* Input:		 none
* Output:		 none
* Logic:		 Initialse timer1
* Example call:	 timer1_init();
*******************************************************************************************************************************/
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
/******************************************************************************************************************************
* Function Name: servo_1
* Input:		 degrees
* Output:		 none
* Logic:		 Rotates servo1 to specified degrees
* Example call:	 servo_1(90);
*******************************************************************************************************************************/
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}
//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

/******************************************************************************************************************************
* Function Name: servo_1_free
* Input:		 none
* Output:		 none
* Logic:		 Frees the servo1
* Example call:	 servo_1_free();
*******************************************************************************************************************************/
void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

/******************************************************************************************************************************
* Function Name: servo
* Input:		 degrees
* Output:		 none
* Logic:		 Rotate servo from current degree to specified degree
* Example call:	 servo(60);
*******************************************************************************************************************************/
void servo(int degrees)
{
	int i;
	if(degrees > c_degrees)
		for (i = c_degrees; i <degrees; i++)
		{
			servo_1(i);
			_delay_ms(20);
			c_degrees=i;
		}
	else
		for (i = c_degrees; i > degrees; i--)
		{
			servo_1(i);
			_delay_ms(10);
			c_degrees=i;
		}
}

/******************************************************************************************************************************
* Function Name: first_aid_kit_deposit
* Input:		 none
* Output:		 none
* Logic:		 Deposits First Aid Kit
* Example call:	 first_aid_kit_deposit();
*******************************************************************************************************************************/

void first_aid_kit_deposit(void)
{
	first_aid_kit_cnt++;
	switch(first_aid_kit_cnt)
	{
		case 1:	servo(32);
				break;
		case 2:	servo(70);
				break;
		case 3:	servo(108);
				break;
		case 4:	servo(144);
				break;		
		case 5:	servo(180);
				break;
		default:break;		
	}
	_delay_ms(100);
}

#endif