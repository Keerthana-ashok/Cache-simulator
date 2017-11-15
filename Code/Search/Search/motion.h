/***************************************************************************************************************************************
****************************************************************************************************************************************
 * Team ID:     B34
 * Author List:	Sanjay A C, Vikas H C, Keerthana A B, Sucharitha Bhat J S. 	  
 * Filename:	motion.h
 * Theme:		Search and Rescue
 * Functions:	motion_pin_config,left_encoder_pin_config,right_encoder_pin_config,left_position_encoder_interrupt_init,
				right_position_encoder_interrupt_init,timer5_init,velocity,ISR(INT5_vect),ISR(INT4_vect),motion_set,forward,back,left,
				right,stop,angle_rotate,left_degrees,right_degrees,path_mm,linear_mm,path_node,rotate_right,rotate_left
				
 * Global Variables:   	ShaftCountLeft,		(To Store the Number of Left Shaft Count),
						ShaftCountRight,	(To Store the Number of Left Shaft Count)
						
 *						
 *
 ***************************************************************************************************************************************
 ***************************************************************************************************************************************
 */
#ifndef MOTION_H
#define MOTION_H

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

/******************************************************************************************************************************
* Function Name: motion_pin_config
* Input:		 none
* Output:		 none
* Logic:		 configure ports to enable robot's motion
* Example call:	 motion_pin_config();
*******************************************************************************************************************************/

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/******************************************************************************************************************************
* Function Name: left_encoder_pin_config
* Input:		 none
* Output:		 none
* Logic:		 configure INT4 (PORTE 4) pin as input for the left position encoder
* Example call:	 left_encoder_pin_config();
*******************************************************************************************************************************/

void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/******************************************************************************************************************************
* Function Name: right_encoder_pin_config
* Input:		 none
* Output:		 none
* Logic:		 configure INT5 (PORTE 5) pin as input for the right position encoder
* Example call:	 right_encoder_pin_config();
*******************************************************************************************************************************/

void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/******************************************************************************************************************************
* Function Name: left_position_encoder_interrupt_init
* Input:		 none
* Output:		 none
* Logic:		 initialse left_position_encoder_interrupt
* Example call:	 left_position_encoder_interrupt_init();
*******************************************************************************************************************************/

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli();	//Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

/******************************************************************************************************************************
* Function Name: right_position_encoder_interrupt_init
* Input:		 none
* Output:		 none
* Logic:		 initialse left_position_encoder_interrupt
* Example call:	 right_position_encoder_interrupt_init();
*******************************************************************************************************************************/

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

/******************************************************************************************************************************
* Function Name: timer5_init
* Input:		 none
* Output:		 none
* Logic:		 Timer 5 initialized in PWM mode for velocity control
* Example call:	 timer5_init();
*******************************************************************************************************************************/

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/******************************************************************************************************************************
* Function Name: velocity
* Input:		 left_motor(velocity of left motor(0 - 255)),right_motor(velocity of left motor(0 - 255))
* Output:		 none
* Logic:		 Sets the velocity for lef and right motor
* Example call:	 velocity(250,150);
*******************************************************************************************************************************/

void velocity (unsigned char left_motor, unsigned char right_motor)	// Function for robot velocity control
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/******************************************************************************************************************************
* Function Name: ISR(INT5_vect)
* Input:		 interrupt from right shaft position encoder
* Output:		 none
* Logic:		 increment right shaft position count
* Example call:	 
*******************************************************************************************************************************/

ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}

/******************************************************************************************************************************
* Function Name: ISR(INT4_vect)
* Input:		 interrupt from left shaft position encoder
* Output:		 none
* Logic:		 increment left shaft position count
* Example call:	 
*******************************************************************************************************************************/

ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

/******************************************************************************************************************************
* Function Name: motion_set
* Input:		 Direction
* Output:		 none
* Logic:		 sets motor's direction
* Example call:	 motion_set(0x01);
*******************************************************************************************************************************/

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

/******************************************************************************************************************************
* Function Name: forward
* Input:		 none
* Output:		 none
* Logic:		 moves the robot in forward direction
* Example call:	 forward();
*******************************************************************************************************************************/

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

/******************************************************************************************************************************
* Function Name: back
* Input:		 none
* Output:		 none
* Logic:		 moves the robot in backward direction
* Example call:	 back();
*******************************************************************************************************************************/

void back (void) //both wheels forward
{
	motion_set(0x09);
}

/******************************************************************************************************************************
* Function Name: left
* Input:		 none
* Output:		 none
* Logic:		 moves the robot towards left
* Example call:	 left();
*******************************************************************************************************************************/

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

/******************************************************************************************************************************
* Function Name: right
* Input:		 none
* Output:		 none
* Logic:		 moves the robot towards right
* Example call:	 right();
*******************************************************************************************************************************/

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

/******************************************************************************************************************************
* Function Name: stop
* Input:		 none
* Output:		 none
* Logic:		 stops the robot
* Example call:	 stop();
*******************************************************************************************************************************/

void stop (void)
{
	motion_set(0x00);
}

/******************************************************************************************************************************
* Function Name: angle_rotate
* Input:		 Degrees
* Output:		 none
* Logic:		 Turns robot by specified degrees
* Example call:	 angle_rotate(90);
*******************************************************************************************************************************/

void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
		
	}
	stop(); //Stop robot
}

/******************************************************************************************************************************
* Function Name: left_degrees
* Input:		 Degrees
* Output:		 none
* Logic:		 turns robot towards left by specified degrees
* Example call:	 left_degrees(90);
*******************************************************************************************************************************/

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	velocity(255,255);
	left(); //Turn left
	angle_rotate(Degrees);
}

/******************************************************************************************************************************
* Function Name: right_degrees
* Input:		 Degrees
* Output:		 none
* Logic:		 turns robot towards right by specified degrees
* Example call:	 right_degrees(90);
*******************************************************************************************************************************/

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	velocity(255,255);
	right(); //Turn right
	angle_rotate(Degrees);
}

/******************************************************************************************************************************
* Function Name: path_mm
* Input:		 distance
* Output:		 none
* Logic:		 Moves robot by specified distance along the black path
* Example call:	 path_mm(90);
*******************************************************************************************************************************/

void path_mm (unsigned long int distance)		//Line following cowering a specified linear distance in mm
{
	unsigned char Left_white_line,Center_white_line,Right_white_line;
	unsigned long int dec=distance;
	ShaftCountRight=0;

	forward();
	while((ShaftCountRight * 5.333)<dec)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		if(Center_white_line > 12 )
		{
			forward();
			velocity(255,255);
		}
		else if(Left_white_line > 12)
		{
			forward();
			velocity(100,255);
		}
		else if(Right_white_line > 12)
		{
			forward();
			velocity(255,100);
		}
		
	}
	stop();
	_delay_ms(8);
}

/******************************************************************************************************************************
* Function Name: linear_mm
* Input:		 distance
* Output:		 none
* Logic:		 Moves robot by specified distance
* Example call:	 linear_mm(90);
*******************************************************************************************************************************/

void linear_mm (unsigned long int distance)		//Line following cowering a specified linear distance in mm
{
	unsigned long int dec=distance;
	ShaftCountRight=0;

	//forward();
	while((ShaftCountRight * 5.333)<dec)
	;
	stop();
	_delay_ms(8);
}

/******************************************************************************************************************************
* Function Name: path_node
* Input:		 n (number of nodes)
* Output:		 none
* Logic:		 Moves robot untill 'n' nodes are detected along the black path
* Example call:	 path_node(2);
*******************************************************************************************************************************/

void path_node (int n)			//Line following cowering a specified number of nodes
{
	unsigned char Left_white_line,Center_white_line,Right_white_line,count = 0;
	
	forward();
	velocity(255,255);

	while(count < n)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor	
		
		if(((Center_white_line>12 && Left_white_line>12) ||(Center_white_line>12 && Right_white_line>12)))// Node Detection
		{			
			count++;
			path_mm(90);
			forward();
			velocity(255,255);			
		}
		else
		{
			if(Center_white_line > 12 )
			{
				velocity(255,255);
				forward();
			}
			else if(Left_white_line > 12)
			{
				velocity(100,255);
				forward();
			}
			else if(Right_white_line > 12)
			{
				velocity(255,100);
				forward();
			}
		}
	}
	
	stop();
	_delay_ms(8);
}

void path_node1 (int n)			//Line following cowering a specified number of nodes
{
	unsigned char Left_white_line,Center_white_line,Right_white_line,count = 0;
	
	forward();
	velocity(255,255);

	while(count < n)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		if(((Center_white_line>12 && Left_white_line>12) ||(Center_white_line>12 && Right_white_line>12)))// Node Detection
		{
			
			count++;
			path_mm(50);
			forward();
			velocity(255,255);
		}
		else
		{
			if(Center_white_line > 12 )
			{
				velocity(255,255);
				forward();
			}
			else if(Left_white_line > 12)
			{
				velocity(100,255);
				forward();
			}
			else if(Right_white_line > 12)
			{
				velocity(255,100);
				forward();
			}
		}
	}
	
	stop();
	_delay_ms(8);
}

/******************************************************************************************************************************
* Function Name: rotate_right
* Input:		 Degrees
* Output:		 none
* Logic:		 turns robot towards right untill black line is detected with a offset of specified degrees 
* Example call:	 rotate_right(90);
*******************************************************************************************************************************/

void rotate_right(int Deg)		// Rotate right with finding the black line
{
	velocity(250,250);
	right_degrees(Deg-20);
	
	char i=3;
	while(ADC_Conversion(2)<12)// to detect balck line
	{
		right_degrees(i); 
	}
	stop();
	_delay_ms(8);
}

/******************************************************************************************************************************
* Function Name: rotate_left
* Input:		 Degrees
* Output:		 none
* Logic:		 turns robot towards left untill black line is detected with a offset of specified degrees 
* Example call:	 rotate_left(90);
*******************************************************************************************************************************/

void rotate_left(int Deg)		// Rotate right with finding the black line
{
	velocity(250,250);
	left_degrees(Deg-20);	
	
	char i=3;
	while(ADC_Conversion(2)<12)	// to detect balck line
	{
		left_degrees(i);
	}
	stop();
	_delay_ms(8);
}

#endif