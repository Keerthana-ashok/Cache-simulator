/***************************************************************************************************************************************
****************************************************************************************************************************************
 * Team ID:     eYRCPlus-SR#935
 * Author List:	Sanjay A C, Vikas H C, Keerthana A B, Sucharitha Bhat J S. 	  
 * Filename:	sensor.h
 * Theme:		Search and Rescue
 * Functions:	adc_pin_config,adc_init,ADC_Conversion,Sharp_GP2D12_estimation,detect_obstacle
 *						
 *
 ***************************************************************************************************************************************
 ***************************************************************************************************************************************
 */
#ifndef SENSOR_H
#define SENSOR_H

/******************************************************************************************************************************
* Function Name: adc_pin_config
* Input:		 none
* Output:		 none
* Logic:		 configure ports to enable ADC pins of robot
* Example call:	 adc_pin_config();
*******************************************************************************************************************************/

void adc_pin_config (void)			//ADC pin configuration
{
 	DDRF = 0x00; //set PORTF direction as input
 	PORTF = 0x00; //set PORTF pins floating
 	DDRK = 0x00; //set PORTK direction as input
 	PORTK = 0x00; //set PORTK pins floating
}

/******************************************************************************************************************************
* Function Name: adc_init
* Input:		 none
* Output:		 none
* Logic:		 Initailise ADC pins of robot
* Example call:	 adc_init();
*******************************************************************************************************************************/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/******************************************************************************************************************************
* Function Name: ADC_Conversion
* Input:		 ch (ADC Channel)
* Output:		 ADC sensor reading of specified channel
* Logic:		 Accepts the Channel Number and returns the corresponding Analog Value 
* Example call:	 ADC_Conversion(2);
*******************************************************************************************************************************/
unsigned char ADC_Conversion(unsigned char Ch)		
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/******************************************************************************************************************************
* Function Name: Sharp_GP2D12_estimation
* Input:		 adc_reading (ADC sensor value)
* Output:		 actual distance in millimeters(mm)
* Logic:		 Calculates the actual distance in millimeters(mm) from the input analog value of Sharp Sensor
* Example call:	 Sharp_GP2D12_estimation(200);
*******************************************************************************************************************************/
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

/******************************************************************************************************************************
* Function Name: detect_obstacle
* Input:		 low(lower limit),high(Upper Limit)
* Output:		 status of obstucle
* Logic:		 detects the obstucle present in front of Front_Sharp_IR)sensor
* Example call:	 detect_obstacle(80,180);
*******************************************************************************************************************************/
int detect_obstacle(int low, int high)			// To detect abstrucle present within the distance range low-high 
{
	unsigned char sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	int value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
		
	if(value>low && value <high)				// block detected
		return 1;
	else
		return 0;
}

#endif