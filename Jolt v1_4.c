//*******************************************************************************************************************
//	Project			:	X-ray controller for Jolt																	*
//	Version			:	1.4																							*
//	Author			:	TT modified by JSC																			*
//	Processor		:	PIC16F886																					*
//	Clock			:	20 MHz, external																			*
//	IDE				:	MPLAB IDE Version 8.30.00.00																*
//	Programmer		:	PICKit 2																					*
//	Last Updated	:	Jan 3, 2012																					*
//  Software Name	:	JOLT v1.4																					*
//*******************************************************************************************************************
// Revision history
// V1.4 12/5/13 DJ - Added screen to show estimated time to complete all pulses. Deleted alot of dead code and unused
// literals to increase available code space. Inlined Trigger_Single_X_Ray() to eliminate warning about possible 
// stack overflow. The pic has a fixed 8 level stack and the compiler was warning of possible stack overflow.
// Modified Trigger_Single_X_Ray() to alway detect pulse length. 
// 

#define _LEGACY_HEADERS
#include	<pic.h>  

__CONFIG(PROTECT & PWRTEN & HS & BORDIS & WDTDIS & LVPDIS & IESODIS );					//	Address 0x2007
__CONFIG(BORV21);																		//	Address 0x2008

/*	
**	PIC 16F886 Configuration
**	UNPROTECT	-	No program code protection
**	PWRTEN		-	Enable power up timer
**	HS			-	Oscillator Selection: High speed crystal
**	BORDIS		-	Disable brown out reset
**	WDTDIS		-	Disable watchdog timer
**	LVPDIS		-	Low voltage programming disabled
**	IESODIS		-	Internal/ External switchover disable
**	BORV21		-	Brown out reset @ 2.1 volts
*/
  
#define bitset(var,bitno) ((var) |= 1 << (bitno))
#define bitclr(var,bitno) ((var) &= ~(1 << (bitno)))

#define 	TRUE		1
#define 	FALSE		0
#define		USER_CANCEL	3
#define		OFF			0
#define		ON			1

#define		COL1		RB0	
#define		COL2		RB1	
#define		COL3		RB2	

#define		RELAY		RA1
#define 	LED			RB4

#define		E			RA5	
#define		RW			RA3	
#define		BLITE		RA4
#define		RS			RA2	

#define		BUZZ1		RC4			
#define		BUZZ2		RC5

#define		CLR_DISP		0x01	
#define		DISP_ON			0x0C
#define		CUR_ON_BLINK    0x0F
#define		DISP_OFF	    0x08

#define		DD_RAM_ADDR1	0x80			//	First line of LCD	
#define		DD_RAM_ADDR2	0xC0			//	Second line of LCD
#define		DD_RAM_ADDRP1   0x87			//	First digit of pulse
#define		DD_RAM_ADDRP2   0x88			//	Second digit of pulse
#define		DD_RAM_ADDRP3   0x89			//	Third digit of pulse
#define		DD_RAM_ADDRP4   0x8A			//	Fourth digit of pulse
#define		DD_RAM_ADDRD1   0xC7			//	First digit of delay
#define		DD_RAM_ADDRD2 	0xC9			//	Second digit of delay
#define		DD_RAM_ADDRD3 	0xCA			//	Third digit of delay
#define		DD_RAM_ADDRP 	0xC8			//	Point digit of delay

#define		ASCII_ZERO		0x30
#define		ASCII_ONE		0x31
#define		ASCII_TWO		0X32
#define		ASCII_THREE		0x33
#define		ASCII_NINE		0x39

#define		ASCII_D			0x44
#define		ASCII_O			0x4F
#define		ASCII_N			0x4E
#define		ASCII_E			0x45
#define		ASCII_S			0x53
#define		ASCII_T			0x54
#define		ASCII_P			0x50

#define		DELAY_250us		61				//	Depends on the compiler, for this setting Delay(61) is 250us
#define		DELAY_500us		123				//	Delay(123) is 498us
#define		DELAY_2ms		498				//	Delay(498) is 1.9468ms

#define		MAX_INDEX		7				//	Index for pulses and delay (4 for pulses and 3 for delay)

enum buttonlist {BUTTON_NONE, BUTTON1_ON, BUTTON1_OFF, BUTTON2_ON, BUTTON2_OFF, BUTTON3_ON, BUTTON3_OFF,
             	BUTTON4_ON, BUTTON4_OFF, BUTTON5_ON, BUTTON5_OFF, BUTTON6_ON, BUTTON6_OFF};

enum BUTTON_LIST {BUTTON_NO, BUTTON_UP, BUTTON_DOWN, BUTTON_LEFT, BUTTON_RIGHT, BUTTON_ENTER, BUTTON_CANCEL};

unsigned char B1, B2, B3, B4, B5, B6;

enum xray_modes{SAFE,DELAYED,FAST};

////////////////////////////////////////////////////////////////////////////////////////////////////
//unsigned int mode1 = 0;						//	Mode 1: FAST SAFE MODE - Fire 50 pulses and stop for 10 secs
//unsigned int mode2 = 0;						//	Mode 2: FAST DANGER MODE - Fire 50 pulses and stop for 5 secs
//unsigned int mode3 = 0;						//	Mode 3: REGULAR MODE - Fire as user entered.

const unsigned char RSLMsg1[]  = "    JOLT II     ";  		//**********4 pins**********
const unsigned char RSLMsg2[]  = " By RSL V1.4    ";  
const unsigned char RSLMsg3[]  = "PULSES";
const unsigned char RSLMsg4[]  = "DELAY   .    sec";
const unsigned char RSLMsg5[]  = "WARNING: Prepare";  
const unsigned char RSLMsg6[]  = "to STEP BACK 10'";
const unsigned char RSLMsg7[]  = "***** FIRE *****";
const unsigned char RSLMsg8[]  = "     Please     ";
const unsigned char RSLMsg9[]  = " STEP BACK 10FT ";        
const unsigned char RSLMsg14[] = "FAST SAFE MODE  ";
const unsigned char RSLMsg15[] = "FAST DANGER MODE";
const unsigned char RSLMsg16[] = "FEEDBACK ERROR!!";
const unsigned char RSLMsg19[] = "C-Mode F-Select";
//const unsigned char RSLMsg20[] = "4...3...2...1...";
const unsigned char RSLMsg21[] = " Estimated Time";

unsigned char cd[16] = {0x34,0x2E,0x2E,0x2E,0x33,0x2E,0x2E,0x2E,0x32,0x2E,0x2E,0x2E,0x31,0x2E,0x2E,0x2E};		//	Default 0000 pulses and 1.00 sec delay

// X-Ray Characteristic	
unsigned char X_Ray_Type = 0;			// On power sets the trigger hold time to zero, so that the Find_X_Ray_Type will be tested
unsigned char X_Ray_Mode = 0;			// The mode that the xrays will be taken	
int Global_Pulse_Count = 0;			// The number of xrays that will be taken
unsigned  int numdelay = 100;				//	Delay in ms entered by user

// ISR counters
int Trigger_Count = 0;					// used to set the number of isr's before hitting zero
int Check_Buttons_Timer = 0;
int Comparator_Timeout_Counter = 0;		// used to timeout trigger from comparator
int Delay_Counter = 0;			// used to countout cool down dealy
int Backlight_Timer = 0;				// used to time out how long the backlight stays on

// Quick Button Scan Column	
unsigned char Quick_Button_Scan_Column = 0;			// On power sets the trigger hold time to zero, so that the Find_X_Ray_Type will be tested

void Display_Fire_Duration(unsigned int,unsigned int, unsigned int);
void dotanddelay( int );

//	Delay time = (cntr*2)+3 (in us for 20MHz) (???)
//	FULL OPTIMIZATION of C compilator(10 level, enable post-pass optimisation) (???)

/*****************************************************************************
* Function Name: isr
* 
* Description:  Interrupt Handler for Timer1. Updates Timer1 counters
* 				and resets Timer1 interrupt flag. 
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void interrupt isr(void)					
{
	if (TMR1IF && TMR1ON)												// check if Timer1 interrupt enabled
	{																	// check if Timer1 interrupt flag set
		if(Comparator_Timeout_Counter > 0) Comparator_Timeout_Counter--;// count down until feedback error							
		if(Check_Buttons_Timer > 0) Check_Buttons_Timer--;							// count down until buttons are reset and ready to be checked
		if(Trigger_Count > 0) Trigger_Count--;							// count down length of xray trigger hold
		if(Delay_Counter > 0) Delay_Counter--;		// count down the length of coold down	
		if(Backlight_Timer > 0) Backlight_Timer--;						// count down the lenght of backlight timer	

		TMR1IF = 0;				// Clear the interrupt flag of timer1				
		TMR1H = 0xF3;			// 5ms is 65535 - (5ms/1.6us) = 62410(0xF3CA)
		TMR1L = 0xCA;
		//TMR1ON = 1;				// Turn on timer1
	}
}

/*****************************************************************************
* Function Name: Init_TMR1()
* 
* Description:  Turn on interrupt Handler for Timer1. 
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Init_TMR1(void)					
{		TMR1IE = 1;				// enable interrupts
	PEIE = 1;
	GIE = 1;
	//TMR1IF = 0;				// Clear the interrupt flag of timer1
	TMR1ON = 1;				// Turn on timer1
}

void Delay(unsigned int cntr) 
{	
	unsigned  int dly;

	for (dly=0; dly<cntr; dly++) 
	{	
		asm("nop");
		asm("nop");	
		asm("nop");	
	}
}

void DelayMs(unsigned int t)
{
	int i;
	
	for(i=0; i<t; i++)
	{	
		Delay (DELAY_250us);
		Delay (DELAY_250us);
		Delay (DELAY_250us);
		Delay (DELAY_250us);
	}
}

void E_Pulse(void) 
{	
	E=1;
	asm("nop");
	asm("nop");
	asm("nop");
	E=0;
}

//	Init LCD  after reset
void InitLCD(void) 
{	
	INTCON 	= 0;			// Disable interrupt
	TRISA2 	= 0;			// RS pin as output							
	TRISA3 	= 0;			// RW pin as output							
	TRISA5 	= 0;			// E pin as output	
						
	RS=0;				
	RW=0;
	DelayMs(213);
	PORTC=0b00000011;		
	E_Pulse();
	DelayMs(18);	
	PORTC=0b00000011;
	E_Pulse();
	DelayMs(18);
	PORTC=0b00000011;
	E_Pulse();
	DelayMs(18);
	PORTC=0b00000010;
	E_Pulse();
}

//	Send char to LCD
LCDSendChar(unsigned char c) 
{	
	unsigned  char data;	

	Delay(DELAY_2ms);
	//	Get upper nibble
	data = c & 0b11110000;		
	//	Set D4-D7	
	data = data >> 4;			
	//	Send data to LCD
	PORTC = data;				
	//	Set LCD to write
	RW=0;						
	//	Set LCD to data mode
	RS=1;						
	//	Toggle E for LCD
	E_Pulse();
	//	Get lower nibble
	data = c & 0b00001111;
	//	Send data to LCD
	PORTC = data;
	//	Set LCD to write
	RW=0;			
	//	Set LCD to data mode			
	RS=1;						
	//	Toggle E for LCD
	E_Pulse();
}

//	Send command to LCD
LCDSendCmd(unsigned char c) 
{	
	unsigned  char data;	

	Delay(DELAY_2ms);
	//	Get upper nibble
	data = c & 0b11110000;		
	//	Set D4-D7	
	data = data >> 4;			
	//	Send data to LCD
	PORTC = data;				
	//	Set LCD to write
	RW=0;						
	//	Set LCD to data mode
	RS=0;						
	//	Toggle E for LCD
	E_Pulse();
	//	Get lower nibble
	data = c & 0b00001111;
	PORTC = data;
	//	Set LCD to write
	RW=0;						
	//	Set LCD to data mode
	RS=0;						
	//	Toggle E for LCD
	E_Pulse();
}


/*****************************************************************************
* Function Name: 

* 
* Description:  This function checks two of the buttons then sets up the ports 
*					for the next set of buttons that are going to be checked, so 
*					that the program isn't delay for 2ms, like with	char ButtonScan(void) 
*
* Arguments: None
*
* Returns: button, from enum BUTTONLIST {BUTTON_NONE, BUTTON_UP, BUTTON_DOWN, BUTTON_LEFT, BUTTON_RIGHT, BUTTON_ENTER, BUTTON_CANCEL;
*
*******************************************************************************/
char Scan_Buttons(void) 
{		
	PORTB |= 0x07;					
	TRISB |= 0x07;
	bitclr(TRISB, 0);	 		//	Set COL1 ---------------------------------------------	
	DelayMs(2);
	
	//	Button 1 is pressed		
	if((COL2==1)&&(B1==OFF)) { B1=ON; return BUTTON_UP; }
	if((COL2==0)&&(B1==ON))  { B1=OFF; return BUTTON_NO; }
	
	//	Button 6 is pressed
	if((COL3==1)&&(B6==OFF)) { B6=ON; return BUTTON_CANCEL; }	
	if((COL3==0)&&(B6==ON))  { B6=OFF; return BUTTON_NO; }	
	
	PORTB |= 0x07;
	TRISB |= 0x07;
	bitclr(TRISB,1);		//	Set COL2 ---------------------------------------------						
	DelayMs(2);
	
	//	Button 4 is pressed
	if((COL1==1)&&(B4==OFF)) { B4=ON; return BUTTON_RIGHT; }
	if((COL1==0)&&(B4==ON))  { B4=OFF; return BUTTON_NO; }			
	
	//	Button 2 is pressed
	if((COL3==1)&&(B2==OFF)) { B2=ON; return BUTTON_ENTER; }
	if((COL3==0)&&(B2==ON))  { B2=OFF; return BUTTON_NO; }
	
	PORTB |= 0x07;
	TRISB |= 0x03;
	bitclr(TRISB,2);		//	Set COL3 ---------------------------------------------
	DelayMs(2);
	
	//	Button 3 is pressed
	if((COL1==1)&&(B3==OFF)) { B3=ON; return BUTTON_LEFT; }
	if ((COL1==0)&&(B3==ON)) { B3=OFF; return BUTTON_NO; }
	
	//	Button 6 is pressed
	if((COL2==1)&&(B5==OFF)) { B5=ON; return BUTTON_DOWN; }
	if((COL2==0)&&(B5==ON))  { B5=OFF; return BUTTON_NO; }	
	
	return BUTTON_NONE;	
}

/*****************************************************************************
* Function Name: Init_Quick_Button_Scan
* 
* Description:  This function sets up the ports before the first set of buttons
*					is checked, so that the program isn't delay for 2ms, like with
*					char ButtonScan(void) 
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Init_Quick_Button_Scan(void)
{
	//	Set COL1 ---------------------------------------------
	PORTB |= 0x07;
	TRISB |= 0x07;
	bitclr(TRISB, 0);

	Quick_Button_Scan_Column = 0;

	Check_Buttons_Timer = 2;
}

/*****************************************************************************
* Function Name: Quick_Button_Scan
* 
* Description:  This function checks two of the buttons then sets up the ports 
*					for the next set of buttons that are going to be checked, so 
*					that the program isn't delay for 2ms, like with	char ButtonScan(void) 
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
char Quick_Button_Scan(void) 
{	
	if(Quick_Button_Scan_Column == 0){	

		//	Button 1 is pressed		
		if((COL2==1)&&(B1==OFF)) { B1=ON; return BUTTON1_ON; }
		if((COL2==0)&&(B1==ON))  { B1=OFF; return BUTTON1_OFF; }
		
		//	Button 5 is pressed
		if((COL3==1)&&(B5==OFF)) { B5=ON; return BUTTON5_ON; }	
		if((COL3==0)&&(B5==ON))  { B5=OFF; return BUTTON5_OFF; }
		
		//	Set conditions for next set of buttons
		PORTB |= 0x07;
		TRISB |= 0x07;
		bitclr(TRISB,1);
		Quick_Button_Scan_Column = 1;								// Set conidtion to check next set of button on next loop
								
	}else if(Quick_Button_Scan_Column == 1){
	
		//	Button 4 is pressed
		if((COL1==1)&&(B4==OFF)) { B4=ON; return BUTTON4_ON; }
		if((COL1==0)&&(B4==ON))  { B4=OFF; return BUTTON4_OFF; }			
		
		//	Button 3 is pressed
		if((COL3==1)&&(B3==OFF)) { B3=ON; return BUTTON3_ON; }
		if((COL3==0)&&(B3==ON))  { B3=OFF; return BUTTON3_OFF; }
		
		//	Set conditions for next set of buttons
		PORTB |= 0x07;
		TRISB |= 0x03;
		bitclr(TRISB,2);	
		Quick_Button_Scan_Column = 2;								// Set conidtion to check next set of button on next loop
						
	}else if(Quick_Button_Scan_Column == 2){
		
		//	Button 2 is pressed
		if((COL1==1)&&(B2==OFF)) { B2=ON; return BUTTON2_ON; }
		if ((COL1==0)&&(B2==ON)) { B2=OFF; return BUTTON2_OFF; }
		
		//	Button 6 is pressed
		if((COL2==1)&&(B6==OFF)) { B6=ON; return BUTTON6_ON; }
		if((COL2==0)&&(B6==ON))  { B6=OFF; return BUTTON6_OFF; }	

		//	Set conditions for next set of buttons
		PORTB |= 0x07;
		TRISB |= 0x07;
		bitclr(TRISB, 0);	
		Quick_Button_Scan_Column = 0;								// Set conidtion to check next set of button on next loop
	}
	return BUTTON_NONE;	
}

void LCDSendStr(const unsigned char* str) 
{	
	int i;

	i = 0;
	while(str[i]) 
	{	
		LCDSendChar(str[i++]);
	}
}

void Beep(unsigned char t, unsigned int delay)
{	
	while (--t)
	{
		BUZZ1 = OFF;					//	For 16F886, different from 16F876A
		BUZZ2 = ON;					
		Delay(delay);				
		BUZZ1 = ON;
		BUZZ2 = OFF;
	 	Delay(delay);
	}
}

void DisplaySplashScreen(void)
{
	InitLCD();                          		//	Init the LCD
	LCDSendCmd(DISP_ON);
	LCDSendCmd(CLR_DISP);
	LCDSendCmd(DD_RAM_ADDR1);

	LCDSendStr(RSLMsg1);						//	Splash screen at power up for 4 and 5 pins
	LCDSendCmd(DD_RAM_ADDR2);
	LCDSendStr(RSLMsg2);						//	Splash screen second line  "By RSL  V1.0"
	DelayMs(1000);
}

void Digital886(void)
{
	//	Digital for 886
	ANS1=0;						
	ANS2=0;						
	ANS3=0;						
	ANS4=0;					
	ANS8=0;					
	ANS9=0;					
	ANS10=0;				
	ANS11=0;				
	ANS12=0;				
	ANS13=0;				
}





void dotanddelay(int offset)
{
	LCDSendCmd(DD_RAM_ADDR2+offset++);
	LCDSendChar(0x2E);
	DelayMs(100);
	LCDSendCmd(DD_RAM_ADDR2+offset++);
	LCDSendChar(0x2E);
	DelayMs(100);
	LCDSendCmd(DD_RAM_ADDR2+offset);
	LCDSendChar(0x2E);
	DelayMs(100);
}



void Init_Comparator(void)
{
	C1CH0 = 0;						//	Set C12IN2- pin of C1 connects to C1VIN-
	C1CH1 = 1;
	C1R = 1;						//	C1VIN+ connects to C1VREF output
	C1POL = 1;						//	C1OUT logic is interted
	C1OE = 0;						//	C1OUT internal only
	//C1OUT=1;						//	C1VIN+<C1VIN-
	C1ON = 1;						//	Enable the comparator C1
	C1RSEL = 1;						//	C1VREF=CVREF
	VRR = 0;						//	High range
}


void annoysound(void)				// This sound is very annoying, the loudest possible about 4 KHz
{
	char m;
	m = 4;
	while(m--)
	{
		Beep(150,61);  
		Beep(150,31);
		Beep(150,60);  
		Beep(150,30);
	}
}

/*****************************************************************************
* Function Name: Display_Delay_Count
* 
* Description:  Display a count down in corner
*
* Arguments: int delay_count, in secs
*
* Returns: Nothing
*
*******************************************************************************/
void Display_Delay_Count(int delay_count)
{
	if(delay_count <= 9 && delay_count >= 0){
		LCDSendCmd(DD_RAM_ADDRP4+5);			
		LCDSendChar(delay_count+0x30);
	} else {
		LCDSendCmd(DD_RAM_ADDRP4+5);			
		LCDSendChar(0x20);
	}
}
/*****************************************************************************
* Function Name: Display_Pulse_Count
* 
* Description:  Display the given pulse count by breaking it into its places
*
* Arguments: int pulse_count
*
* Returns: Nothing
*
*******************************************************************************/
void Display_Pulse_Count(int pulse_count)
{
	unsigned char thousands_place, hundreds_place, tens_place, ones_place;

	thousands_place = pulse_count / 1000;	// Break pulse_count into individual vars
	hundreds_place = (pulse_count % 1000)/100;
	tens_place = (pulse_count % 100)/10;
	ones_place = pulse_count % 10;

	LCDSendCmd(DD_RAM_ADDRP1);				// First #, MSB, in first place
	LCDSendChar(thousands_place+0x30);		// number conversion to ASCII is +0x30
	
	LCDSendCmd(DD_RAM_ADDRP2);
	LCDSendChar(hundreds_place+0x30);
	
	LCDSendCmd(DD_RAM_ADDRP3);
	LCDSendChar(tens_place+0x30);
	
	LCDSendCmd(DD_RAM_ADDRP4);				// LSB is last place
	LCDSendChar(ones_place+0x30);	

	LCDSendCmd(DD_RAM_ADDRP4+9);			// Move cursor out of the screen for easy to see
}

/*****************************************************************************
* Function Name: Display_Delays
* 
* Description:  Display the given delay, X.XXsecs
*
* Arguments: int time_delay, input is whole number XXX = X.XX
*
* Returns: Nothing
*
*******************************************************************************/
void Display_Delay(int time_delay)
{
	unsigned char ones_place, tenths_place, hundredths_place;

	ones_place = time_delay / 100;
	tenths_place = (time_delay % 100)/10;
	hundredths_place = time_delay % 10;

	LCDSendCmd(DD_RAM_ADDRD1);			//	display delay digits
	LCDSendChar(ones_place+0x30);
	
	LCDSendCmd(DD_RAM_ADDRD2);
	LCDSendChar(tenths_place+0x30);
	
	LCDSendCmd(DD_RAM_ADDRD3);
	LCDSendChar(hundredths_place+0x30);	
}

/*****************************************************************************
* Function Name: Check_Button_Press
* 
* Description:  Check the buttons, requires Init_Quick_Button_Scan(); be called prior
*
* Arguments: None
*
* Returns: button_press, TRUE = button was pressed, else FALSE
*
*******************************************************************************/
char Check_Button_Press(void)
{
	unsigned char button_press, button;
	
	button_press = FALSE;
	
	if(Check_Buttons_Timer <= 0) {						// If the counter is zero its time to check the buttons again
		button = Quick_Button_Scan();				// Check the buttons, because the buttons take a ~2ms to reset we will only check them every 5ms, 15ms for all three columns
				
		if ((button==BUTTON1_ON)||(button==BUTTON2_ON)||(button==BUTTON3_ON)||(button==BUTTON4_ON)||(button==BUTTON5_ON)||(button==BUTTON6_ON)){
			button_press = TRUE;			
			Beep(150,61);
		}

		Check_Buttons_Timer = 2;						// reset check buttons timer		
		
	}
	return(button_press);			
}


/*****************************************************************************
* Function Name: Display_Step_Back
* 
* Description:  Tell user to "Please Step Back"
*
* Arguments: None
*
* Returns: cancelled, if user cancels process
*
*******************************************************************************/
void Display_Step_Back(void)
{
	LCDSendCmd(CLR_DISP);					// Clear display
	LCDSendStr(RSLMsg8);					// "Please"
	LCDSendCmd(DD_RAM_ADDR2);				// Next line
	LCDSendStr(RSLMsg9);					// "Step Back"
}

/*****************************************************************************
* Function Name: Pre_Fire_Beeping
* 
* Description:  Makes beeping sound for 4 seconds
*
* Arguments: None
*
* Returns: cancelled, if user cancels process
*
*******************************************************************************/
void Pre_Fire_Beeping(void)
{	
		unsigned char m=24;                	//	Beeping 4.029151 secs
	while (--m)
	{	
		Beep(150,61);  					//	Count down and fire.
    	LED = ON;
		DelayMs(100);
		LED = OFF;
    }  
  	  
}

/*****************************************************************************
* Function Name: Pre_Fire_Countdown
* 
* Description:  Makes beeping sound for 4 seconds
*
* Arguments: None
*
* Returns: cancelled, if user cancels process
*
*******************************************************************************/
char Pre_Fire_Countdown(void)
{
	unsigned char cancelled, button, loop, pointer;
	int counter;
 	int time_to_completion=0;

	
	cancelled = FALSE;
	loop = TRUE;	

	//Init_Quick_Button_Scan();					// Prepare buttons to be scanned
	Display_Step_Back();						// Display step back
	Delay_Counter = 100;						// .5 Sec countdown
	counter = 0;

	while(loop == TRUE)
	{
		button = Scan_Buttons();			// check for a button push
		
		if(button != BUTTON_NO){						// If button pushed cancel
			loop = FALSE;
			cancelled = TRUE;
		} else if(Delay_Counter == 0){
			Beep(150,61);  						//	Count down and fire.    
			if(counter % 2) LED = ON;			// Blink LED
			else LED = OFF;
			Delay_Counter = 100;				// .5 Sec countdown
			counter++;
		}

		if(counter >= 12) loop = FALSE;
	}

	loop = TRUE;
	Delay_Counter = 50;						// .25 Sec countdown
	counter = 0;
	LCDSendCmd(CLR_DISP);					// Clear display
	
	while(loop == TRUE && cancelled == FALSE)
	{
		button = Scan_Buttons();			// check for a button push
		
		if(button!= BUTTON_NO){				// If button pushed cancel
			loop = FALSE;
			cancelled = TRUE;
		} else if(Delay_Counter == 0){
			Beep(150,61);  						//	Count down and fire.    
			if(counter % 2) LED = ON;			// Blink LED
			else LED = OFF;
			Delay_Counter = 50;					// .25 Sec countdown
			LCDSendChar(cd[counter]);		// "4...3...2...1..."
			counter++;
		}

		if(counter > 16) loop = FALSE;
	}



	return(cancelled);
}

/*****************************************************************************
* Function Name: Display_Mode
* 
* Description:  Display the current mode
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Display_Mode(void)
{
	if(X_Ray_Mode == SAFE){
		LCDSendCmd(DD_RAM_ADDR2);
		LCDSendStr(RSLMsg14);  				//	Display FAST SAFE MODE	
	}else if (X_Ray_Mode == DELAYED){
		LCDSendCmd(DD_RAM_ADDR2);
		LCDSendStr(RSLMsg4);  				//	Display NORMAL MODE
		Display_Delay(numdelay);			// Display the set delay
		LCDSendCmd(DD_RAM_ADDRP1);
	}else if (X_Ray_Mode == FAST){
		LCDSendCmd(DD_RAM_ADDR2);
		LCDSendStr(RSLMsg15);  				//	Display FAST DANGER MODE
	}

}



/*****************************************************************************
* Function Name: Set_Trigger_Length
* 
* Description: Depending on the XRay type(X_Ray_Type) which is a global var, set the 
*				# of 5ms triggers necessary to hold the trigger for the current X_Ray_Type
*				the counter is decrement in the isr
*
* Arguments: None
*
* Returns: trigger_count
*
*******************************************************************************/
void Set_Trigger_Length(void)
{
	if(X_Ray_Type == 1){					// 40ms is 40/5=8
		Trigger_Count = 8;
	} else if(X_Ray_Type == 2){				// 60ms is 60/5=12
		Trigger_Count = 12;
	} else if(X_Ray_Type == 3){				// 140ms is 140/5=28
		Trigger_Count = 28;
	} else if(X_Ray_Type == 4){				// 160ms is 160/5=32
		Trigger_Count = 32;
	} else if(X_Ray_Type == 5){				// 250ms is 250/5=50
		Trigger_Count = 50;
	} else{									// default to lowest setting, user is more concerned with multiple pulses then no pulse
		Trigger_Count = 8;
	} 	
}

void start(void)
{
	OSTS = 1;					//	For 16F886
	
	SCS  = 0;					//	For 16F886
	T0CS = 0;					//	Timer 0, internal instruction cycle clock

	TRISA0 = 1;                 //	Set RA0 as input
	TRISA1 = 0;			        //	Set RELAY pin as output	
	RELAY = OFF;
	ANS0 = 1;					//	Set RA0 as analog (input)
	ADCS0 = 0;					//	For ADC clock = Fosc/32
	ADCS1 = 1;					//	With 20MHz, Tad=1.6us
	VCFG0 = 0;					//	Config voltage reference for ADC
	VCFG1 = 0;					//	as VDD and VSS
	CHS0 = 0;					//	Select AN0 (RA0 as input channel for ADC
	CHS1 = 0;
	CHS2 = 0;
	CHS3 = 0;
	ADFM = 1;					//	Right justified for ADC result
	ADON = 1;					//	Turn on the ADC
	DelayMs(5);					//	Wait for acquistion time, 5ms for 5V
	GODONE = 1;					//	Start the AD conversion	
}



/*****************************************************************************
* Function Name: Feedback_Error THIS COULD BE NEEDING LOOKED AT
* 
* Description:  After a x-ray has been triggered, if the feedback pulse is not detected
*					notify the user of the error
*
* Arguments: None
*
* Returns: confrim, TRUE = A confirmation pulse was detected, else FALSE
*
*******************************************************************************/
void Feedback_Error(void)
{
	while(1)			// notify user of feedback error
	{	
		Beep(150,184);
		DelayMs(100);
		LCDSendCmd(DD_RAM_ADDR2);
		LCDSendStr(RSLMsg16);
	}	
}

/*****************************************************************************
* Function Name: Hold_X_Ray_Trigger
* 
* Description:  Hold the trigger until released
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Hold_X_Ray_Trigger(void)
{
	LED = OFF;									// Turn ON the LED

	RELAY = ON;									// Trigger		
}

/*****************************************************************************
* Function Name: Release_X_Ray_Trigger
* 
* Description:  Release the trigger 
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Release_X_Ray_Trigger(void)
{
	LED = ON;									// Turn off the LED, off is on and on is off

	RELAY = OFF;								// Release Trigger		
}

/*****************************************************************************
* Function Name: Set_Comparator_High_Trigger()
* 
* Description: Set the trigger for the rising edge of the pulse at 3.6V, and
*				switch C1OUT from 0 to 1 when it happens
*				CVREF = Vdd/4 + (VR<3:0>/32)*Vdd
*               assuming CVREF = C1Vin+ & input = C1Vin-			
*				if C1POL = 1 (inverted polarity):
*					C1OUT = 0 when CVREF > input
*					C1OUT = 1 when CVREF < input
*				if C1POL = 0 (non-inverted polarity):
*					C1OUT = 1 when CVREF > input
*					C1OUT = 0 when CVREF < input	
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Set_Comparator_High_Trigger(void)
{
	C1POL = 1;						// C1OUT will equal 1 when, input > CVREF
	
	VR0 = 1;						// With VR<3:0> = 0b1111,
	VR1 = 1;						// CVREF = Vdd/4 + VR<3:0>/32 * Vdd
	VR2 = 1;						// CVREF = (5/4) + ((15/32)*5) = 3.594V
	VR3 = 1;	
}

/*****************************************************************************
* Function Name: Set_Comparator_Low_Trigger()
* 
* Description: Set the trigger for the falling edge of the pulse at 1.75V, and
*				switch C1OUT from 0 to 1 when it happens
*				CVREF = Vdd/4 + (VR<3:0>/32)*Vdd
*               assuming CVREF = C1Vin+ & input = C1Vin-			
*				if C1POL = 1 (inverted polarity):
*					C1OUT = 0 when CVREF > input
*					C1OUT = 1 when CVREF < input
*				if C1POL = 0 (non-inverted polarity):
*					C1OUT = 1 when CVREF > input
*					C1OUT = 0 when CVREF < input	
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Set_Comparator_Low_Trigger(void)
{
	C1POL = 0;						// C1OUT will equal 1 when, input < CVREF

	VR0 = 0;						// With VR<3:0> = 0b0100,
	VR1 = 0;						// CVREF = Vdd/4 + VR<3:0>/32 * Vdd
	VR2 = 1;						// CVREF = (5/4) + ((4/32)*5) = 1.875V
	VR3 = 0;
}

/*****************************************************************************
* Function Name: Init_Detect_Rising_Edge
* 
* Description:  Initalize the comparator for the rising edge of a confirmation pulse.  
*					Trigger when the feedback line goes above 3.6V
*
* Arguments: None
*
* Returns: Nothing
*******************************************************************************/
void Init_Detect_Rising_Edge(void)
{
	Comparator_Timeout_Counter = 58;						// wait for rising edge for 250ms + min trigger time 8ms

	Set_Comparator_High_Trigger();							// Set the comapator to trigger on the rising edge of the pulse
	
	Delay(10);												// 48uSec delay to allow comparator to init
}

/*****************************************************************************
* Function Name: Detect_Rising_Edge
* 
* Description:  Check the comparator for the rising edge of a confirmation pulse.  
*					Trigger when the feedback line goes above 3.6V
*
* Arguments: None
*
* Returns: detected, TRUE = A rising edged confirmation pulse was detected, else FALSE
*
*******************************************************************************/
char Detect_Rising_Edge(void)
{
	unsigned char detected;

	detected = FALSE;

	if(C1OUT == 1) detected = TRUE;				// If the rising edge is detected set flag

	return(detected);
}

/*****************************************************************************
* Function Name: Init_Detect_Falling_Edge
* 
* Description:  Initalize the comparator for the falling edge of a confirmation pulse.  
*					Trigger when the feedback line goes above 1.8V
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Init_Detect_Falling_Edge(void)
{
	Comparator_Timeout_Counter = 30;						// Wait for falling edge for 150ms

	Set_Comparator_Low_Trigger();							// Set the comapator to trigger on the falling edge of the pulse
	
	Delay(10);												// 48uSec delay to allow comparator to init
}

/*****************************************************************************
* Function Name: Detect_Falling_Edge
* 
* Description:  Check the comparator for the falling edge of a confirmation pulse.  
*					Trigger when the feedback line goes above 1.8V
*
* Arguments: None
*
* Returns: detected, TRUE = A rising edged confirmation pulse was detected, else FALSE
*
*******************************************************************************/
char Detect_Falling_Edge(void)
{
	unsigned char detected;

	detected = FALSE;

	if(C1OUT == 1) detected = TRUE;				// If the rising edge is detected set flag

	return(detected);
}



/*****************************************************************************
* Function Name: Confirm_X_Ray
* 
* Description:  after an x-ray has been triggered check the comparators for a 
*					confirmation pulse.  A confirmation will trigger a comparator 
*					on its way up at 4v, then a few ms later on its way down at 1.75V
*
* Arguments: None
*
* Returns: confrim, TRUE = A confirmation pulse was detected, USER_CANCEL = user pressed button, else FALSE
*
*******************************************************************************/
char Confirm_X_Ray(void)
{
	unsigned char button, confirm, cancelled, rising_trigger, falling_trigger;
	
	confirm = FALSE;
	cancelled = FALSE;
	rising_trigger = FALSE;
	falling_trigger = FALSE;	

	Init_Quick_Button_Scan();										// Initialize buttons
	Init_Detect_Rising_Edge();										// Init system to detect rising edge

	while(rising_trigger == FALSE && cancelled == FALSE) {
		
		rising_trigger = Detect_Rising_Edge();						// If the rising edge is detected set flag					

		if(Comparator_Timeout_Counter <= 0 ) break;//Feedback_Error();		// If a rising edge is not detected in the time alotted the there is an error	
				
		cancelled = Check_Button_Press();			
	}
	
	if(rising_trigger == TRUE) {
		Init_Detect_Falling_Edge();									// Set the comapator to trigger on the falling edge of the pulse
		//cancelled = FALSE;

		while(falling_trigger == FALSE && cancelled == FALSE) {
			
			falling_trigger = Detect_Rising_Edge();					// If the falling edge is detected set flag
	
			if(Comparator_Timeout_Counter <= 0 )break;//Feedback_Error();	// If a falling edge is not detected in the time alotted the there is an error
			
			cancelled = Check_Button_Press();						
		}		
	}
	
	if(rising_trigger == TRUE && falling_trigger == TRUE) confirm = TRUE;		// If both the rising and falling edges of the feedback pulse are detected confrim the x-ray was made
	else if(cancelled == TRUE) confirm = USER_CANCEL;							// set flag so other functions know user cancelled operations	
	else confirm = FALSE;	

	return(confirm);
}

/*****************************************************************************
* Function Name: Xray_Delay
* 
* Description:  input a time ms(incremments of 5ms)
*
* Arguments: delay counter, every incriment will count as 5ms
*
* Returns: cancelled, if user presses button cancel
*
*******************************************************************************/
char Xray_Delay(unsigned int delay_counter)
{
	unsigned char continue_delay, cancelled, button;

	Init_Quick_Button_Scan();					// Initialize buttons
	continue_delay = TRUE;
	cancelled = FALSE;

	Delay_Counter = delay_counter*2;		// convert from X.XX sec to # of 5ms ticks

	while(continue_delay)											// Loop until the delay hold is done
	{
		if(Delay_Counter <= 0){	
			continue_delay = FALSE;		// Clear flag to exit the loop	
		} else {
			cancelled = Check_Button_Press();	

			if(cancelled) continue_delay = FALSE;						// Clear flag to exit the loop					
		}
	}	

	return(cancelled);
}

/*****************************************************************************
* Function Name: Overheat_Delay SHOW AN ERROR
* 
* Description:  if mode = 1 delay 10s, if mode = 3 delay 5 sec
*
* Arguments: None
*
* Returns: cancelled, if user presses button cancel
*
*******************************************************************************/
char Overheat_Delay(void)
{
	unsigned char continue_delay, cancelled, button, counter, old_counter;

	Init_Quick_Button_Scan();										// Initialize buttons
	continue_delay = TRUE;
	cancelled = FALSE;

	if (X_Ray_Mode == SAFE) {
		Delay_Counter = 2000;								// Delay 10s, counter = 10s/5ms = 2000
	} else if (X_Ray_Mode == DELAYED) {
		Delay_Counter = 0;									// Delay 0s, mode2 doesnt require a delay
	}else if (X_Ray_Mode == FAST) {
		Delay_Counter = 1000;								// Delay 5s, counter = 5s/5ms = 1000
	}

	while(continue_delay == TRUE)											// Loop until the delay hold is done
	{
		counter = Delay_Counter/200;
		
		if(counter != old_counter) {
			Display_Delay_Count(counter);		
			old_counter = counter;
		}

		if(Delay_Counter <= 0)	continue_delay = FALSE;				// Clear flag to exit the loop	
		
		cancelled = Check_Button_Press();	
		if(cancelled == TRUE) continue_delay = FALSE;						// Clear flag to exit the loop	
		
	}		

	Display_Delay_Count(10);												// clears display

	return(cancelled);
}

/*****************************************************************************
* Function Name: Pulse_Trigger
* 
* Description:  Hold the trigger for a set a mount of time, monitor for cancellation
*
* Arguments: None
*
* Returns: cancelled, TRUE = a button was pressed that caused the trigger to be cancelled, else FALSE
*
*******************************************************************************/
char Pulse_Trigger(void)
{
	unsigned char hold_trigger, cancelled, early_feedback, confirmed;
	int trigger_count;

	hold_trigger = TRUE;								// set trigger hold flag to True, will change to false when trigger is done
	cancelled = FALSE;									// default return that trigger was not cancelled
	early_feedback = FALSE;								// 

	Init_Quick_Button_Scan();							// Initialize buttons
	Init_Detect_Rising_Edge();							// Init system to detect rising edge

	Set_Trigger_Length();								// Set the number of timer interrupts the trigger will require, timer interrupts will be set to 5ms

	Hold_X_Ray_Trigger();								// Hold trigger	

	while(hold_trigger)									// Loop until the tigger hold is done
	{
		early_feedback = Detect_Rising_Edge();			// ERROR: Look for a confirmation pulse earlier then expected to stop trigger
		
		cancelled = Check_Button_Press();				// Check buttons for user cancel

		if(Trigger_Count <= 0 || early_feedback == TRUE || cancelled == TRUE) {
			hold_trigger = FALSE;
		}		
	}		
	
	Release_X_Ray_Trigger();							// Release trigger	

	return(cancelled);
} 

/*****************************************************************************
* Function Name: Find_X_Ray_Type
* 
* Description:  Determine the proper trigger length for the attached x-ray, by
*					starting at a low trigger time and gradually increasint it until
*					a pulse is detected
*
* Arguments: None
*
* Returns: cancelled, if user cancels process
*
*******************************************************************************/
////////Stack Overflow error caused by this routine. This routine was inlined in
// Trigger_Single_X_Ray()
//?????????????????????????????????????????????????????????
//char Find_X_Ray_Type(void)
//{
//	unsigned char exit_loop, cancelled, xray_confirm;
//	exit_loop = FALSE; 											// init vars
//	cancelled = FALSE;
//	xray_confirm = FALSE;
//	X_Ray_Type = 1;						
//	
//
//	while(exit_loop == FALSE)				
//	{		
//		cancelled = Pulse_Trigger();							// if true a early confirmation pulse was detected, false
//
//		if(cancelled == TRUE){
//			exit_loop = TRUE;
//		} else {
////			xray_confirm = Confirm_X_Ray();						// monitor for confirmation pulse
//
//			if(xray_confirm == TRUE){ 							// if a confirmation pulse was detected, then keep X_Ray_Type
//				exit_loop = TRUE;
//				cancelled = FALSE;
//			} else if(xray_confirm == USER_CANCEL){				// if the user cancelled while looking for confirmation pulse
//				exit_loop = TRUE;
//				cancelled = TRUE;
//			} else if(xray_confirm == FALSE){
//				X_Ray_Type++;									// try next type of xray
//				if(X_Ray_Type > 5) Feedback_Error();			// if all types have been tried, give error	
//			}
//		}
//	}
//		
//	return(cancelled);
//}
//
/*****************************************************************************
* Function Name: Trigger_Single_X_Ray
* 
* Description:  Hold the trigger for a set a mount of time, monitor for cancellation and confirmation
*
* Arguments: None
*
* Returns: confrimed, TRUE = A confirmation pulse was detected, USER_CANCEL = user pressed button, else FALSE
*
*******************************************************************************/
char Trigger_Single_X_Ray(void)
{
	unsigned char cancelled,xray_confirm;
	unsigned char exit_loop;
	exit_loop = FALSE; 											// init vars
	cancelled = FALSE;
	xray_confirm = FALSE;
		
	
	xray_confirm = FALSE;
	cancelled = FALSE;

//	if(X_Ray_Type == 0) {
//		Find_X_Ray_Type was inlined below to prevent possible stack overflow.
//		cancelled = Find_X_Ray_Type();				// if the X_Ray_Type is unknown, then Jolt needs to figure it out

		X_Ray_Type = 1;
		while(exit_loop == FALSE)										//modified to always detect single pulse trigger length
		{		
				cancelled = Pulse_Trigger();							// if true a early confirmation pulse was detected, false
		
				if(cancelled == TRUE){
					exit_loop = TRUE;
				} else {
					xray_confirm = Confirm_X_Ray();						// monitor for confirmation pulse
		
					if(xray_confirm == TRUE){ 							// if a confirmation pulse was detected, then keep X_Ray_Type
						exit_loop = TRUE;
						cancelled = FALSE;
					} else if(xray_confirm == USER_CANCEL){				// if the user cancelled while looking for confirmation pulse
						exit_loop = TRUE;
						cancelled = TRUE;
					} else if(xray_confirm == FALSE){
						X_Ray_Type++;									// try next type of xray
						if(X_Ray_Type > 5) Feedback_Error();			// if all types have been tried, give error	
					}
				}
			}


		xray_confirm = TRUE;

		LCDSendCmd(CLR_DISP);						//	Display the number of pulses and delay user entered
		LCDSendStr(RSLMsg3);
		Display_Mode();								// Display the current mode
		
//	} else {
//		cancelled = Pulse_Trigger();							// if true a early confirmation pulse was detected, false
//
//		if(cancelled == FALSE){		
//			xray_confirm = Confirm_X_Ray();						// monitor for confirmation pulse
//		}
//	}

	if(cancelled == TRUE) xray_confirm == USER_CANCEL;

	return(xray_confirm);
}

/*****************************************************************************
* Function Name: Display_Prepare_to_Fire
* 
* Description:  Tell user to "Please Step Back"
*
* Arguments: None
*
* Returns: cancelled, if user cancels process
*
*******************************************************************************/
void Display_Prepare_to_Fire(void)
{
	unsigned char n=15;	

	LCDSendCmd(CLR_DISP);
	LCDSendStr(RSLMsg5);		//	Warning: STEP BACK 10FT with beeping sound
	LCDSendCmd(DD_RAM_ADDR2);
	LCDSendStr(RSLMsg6);
	
	while (n--)
	{
		Beep(75,123);
		Delay(90);
	}
}

/*****************************************************************************
* Function Name: Display_Estimated_Time DO THIS BEFORE STEP BACK MTFKA , ALSO NEEDS TO BE MORE ACCURATE IF LONGER THAN 6 MIN
* 
* Description:  Calculate the amount of time it will take to fire 
*
* Arguments: None
*
* Returns: cancelled, if user cancels process
*
*******************************************************************************/
void Display_Estimated_Time(void)
{
	unsigned char n=15;	
	unsigned int time=0;
	unsigned int groups=0, remainder=0;
	unsigned int hours, min;
	int delay;

	// the unit fires at a rate of 15 pulses per second
	// unsigned char X_Ray_Type = 0;		// On power sets the trigger hold time to zero, so that the Find_X_Ray_Type will be tested
	// unsigned char X_Ray_Mode = 0;		// The mode that the xrays will be taken	
	// int Global_Pulse_Count = 0;			// The number of xrays that will be taken

	// xray mode 1-safe     10 seconds between groups of 50
	// xray mode 2-delayed	
	// xray mode 3-danger   5 seconds between groups of 50
	//enum xray_modes{SAFE,DELAYED,FAST};
	
	// Set the delay time
	switch (X_Ray_Mode){
	case SAFE:
		delay = 10;
		break;
	case FAST:
		delay = 5;
		break;
	default:
		delay = 1;
	}

	// calculate the time for all pulses to be fired
	switch (X_Ray_Mode){
	
	// safe 10 seconds between groups of 50
	case SAFE:
	case FAST:
		 if (Global_Pulse_Count <= 50){
			time = Global_Pulse_Count / 15;			// fires at a rate  of 15/sec
		 	if (time == 0 && Global_Pulse_Count > 0)	// takes care of case Global_Pulse_Count <15
				time = 1;
		 }
		 else{
			groups = Global_Pulse_Count / 50; 		// find out how many groups of 50 there are
			remainder = Global_Pulse_Count%50;		// fractional part of last group
			if ( groups >= 1 && remainder ==0 ){
				time = ((groups-1)*delay) + (groups *33)/10 + remainder/15;  // delay time + shot time + pulses left
			}
			else if ( groups >= 1 && remainder !=0 ){
				time = (groups*delay) + (groups *33)/10 + remainder/15;  // delay time + shot time + pulses left
			}	
			else {
				time = remainder/15;
				if (time == 0) time = 1;
			}
		 }
		break;
	
	// user programmed delay between each pulse
	case DELAYED:
			time=(unsigned int) (((long)numdelay * (long)(Global_Pulse_Count-1))/100);
			//time = numdelay/100 * (Global_Pulse_Count-1);		// numdelay is in ms

		break;
	default:
		time = 0;
		break;
	
	}
	 
	hours=min=0;

	if ( time >= 360 ){
		hours = time / 360;
		time = time % 360;
	}
	if (time >= 60 ){
			min = time / 60;
			time = time % 60;
	}
	
	
	
	LCDSendCmd(CLR_DISP);
	LCDSendStr(RSLMsg21);		//	
	LCDSendCmd(DD_RAM_ADDR2);
	
	Display_Fire_Duration( hours, min, time);
	
}




void Display_Fire_Duration( unsigned int hours, unsigned int min, unsigned int sec){

	unsigned char thousands_place, hundreds_place, tens_place, ones_place;
	unsigned int zero_enable=0;

	thousands_place = hours / 1000;			// Break pulse_count into individual vars
	hundreds_place = (hours % 1000)/100;
	tens_place = (hours % 100)/10;
	ones_place = hours % 10;

	LCDSendCmd(DD_RAM_ADDR2);				// First #, MSB, in first place
	if (thousands_place != 0){
		LCDSendCmd(DD_RAM_ADDR2+2);				// First #, MSB, in first place
		LCDSendChar(thousands_place+0x30);		// number conversion to ASCII is +0x30
		zero_enable = 1;
	}		
	
	if ( hundreds_place != 0 || zero_enable == 1){
		LCDSendCmd(DD_RAM_ADDR2+3);
		LCDSendChar(hundreds_place+0x30);
		zero_enable = 1;
	}
	
	if ( tens_place != 0|| zero_enable == 1 ){
		LCDSendCmd(DD_RAM_ADDR2+4);
		LCDSendChar(tens_place+0x30);
	}
	
	LCDSendCmd(DD_RAM_ADDR2+5);				// LSB is last place
	LCDSendChar(ones_place+0x30);	


	tens_place = (min % 100)/10;
	ones_place = min % 10;

	LCDSendCmd(DD_RAM_ADDR2+6);
	LCDSendChar(':');
	
	
	LCDSendCmd(DD_RAM_ADDR2+7);
	LCDSendChar(tens_place+0x30);
	
	LCDSendCmd(DD_RAM_ADDR2+8);				// LSB is last place
	LCDSendChar(ones_place+0x30);	

	LCDSendCmd(DD_RAM_ADDR2+9);
	LCDSendChar(':');
	
//
	tens_place = (sec % 100)/10;
	ones_place = sec % 10;
	
	LCDSendCmd(DD_RAM_ADDR2+10);
	LCDSendChar(tens_place+0x30);
	
	LCDSendCmd(DD_RAM_ADDR2+11);				// LSB is last place
	LCDSendChar(ones_place+0x30);	

	LCDSendCmd(DD_RAM_ADDR2+19);			// Move cursor out of the screen for easy to see
}

/*****************************************************************************
* Function Name: Display_Done
* 
* Description: Display Done and the # of pulse and mode that was accomplished
*
* Arguments: number of pulses
*
* Returns: Nothing
*
*******************************************************************************/
void Display_Done(int pulses)
{
	LCDSendCmd(CLR_DISP);							//	Display the number of pulses user entered
	LCDSendStr(RSLMsg3);							// Displays Pulses
	Display_Pulse_Count(pulses);					// Display the number of pulses remaining

	LCDSendCmd(DD_RAM_ADDRP4+2);					// Display "DONE:
	LCDSendChar(ASCII_D);
	LCDSendCmd(DD_RAM_ADDRP4+3);
	LCDSendChar(ASCII_O);
	LCDSendCmd(DD_RAM_ADDRP4+4);
	LCDSendChar(ASCII_N);
	LCDSendCmd(DD_RAM_ADDRP4+5);
	LCDSendChar(ASCII_E);

	Display_Mode();									// Display the current mode
	
	annoysound();									// Make annoy sound after finish firing
	annoysound();
	annoysound();
	annoysound();
	annoysound();
}

/*****************************************************************************
* Function Name: Display_Select
* 
* Description: Display "X-SELECT O-ENTER" in top left corner, used during Select_Mode()
*
* Arguments: None
*
* Returns: Nothing
*
*******************************************************************************/
void Display_Select(void)
{
	LCDSendCmd(DD_RAM_ADDR1);						// Display "X-SELECT O-ENTER"
	LCDSendStr(RSLMsg19);							//
}

/*****************************************************************************
* Function Name: Display_Cancelled
* 
* Description: Display Cancelled and the # of pulse remaining and mode
*
* Arguments: number of pulses
*
* Returns: Nothing
*
*******************************************************************************/
void Display_Cancelled(int pulses)
{	
	LCDSendCmd(CLR_DISP);								// Display the number of pulses user entered
	LCDSendStr(RSLMsg3);								// "Pulses"
	Display_Pulse_Count(pulses);						// Display the number of pulses remaining
					
	LCDSendCmd(DD_RAM_ADDRP4+2);						// Display "STOP" after cancelling fire
	LCDSendChar(ASCII_S);
	LCDSendCmd(DD_RAM_ADDRP4+3);
	LCDSendChar(ASCII_T);
	LCDSendCmd(DD_RAM_ADDRP4+4);
	LCDSendChar(ASCII_O);
	LCDSendCmd(DD_RAM_ADDRP4+5);
	LCDSendChar(ASCII_P);
			
	Display_Mode();										// Display the current mode		
	LCDSendCmd(DD_RAM_ADDRP4+9);						// Move cursor out of the screen for easy to see

	annoysound();										// Make annoy sound after cancelling fire
	DelayMs(200);
							

}



/*****************************************************************************
* Function Name: Fire_Mode_2
* 
* Description:  Fire mode e execute a set number of continous xrays, breaking
*				after each xray.  The system triggers a single xray then waits for the 
*				confirmation.  Before triggering the first xray the system determines the 
*				type of xray system connected.  The mininum delay between xrays is 1 sec to 
*				account for 800ms reset time.
*				NOTE: You can not get a continous set of xrays by pressing and releasing the 
*					the trigger, because the xray system will not trigger again for 800ms.
*					Also, if the trigger is just held down and no cool down period is used
*					the xray system will automaticaly stop after 99 xrays.
*
* Arguments: pulses, the number of xrays to take
*
* Returns: done, TRUE = finished, FALSE = didnt finish
*
*******************************************************************************/
char Fire_Modes_2(int pulses)
{
	unsigned char take_xrays, cancelled, xray_confirmed,done;
	unsigned int overheat_count;
				
	// initialize local vars
	take_xrays = TRUE;					// TRUE = take more xrays, FALSE = stop			
	cancelled = FALSE;					// TRUE = user requests stop, FALSE = continue
	xray_confirmed = FALSE;				// TRUE = xray feeback confirmed, FALSE = no xray confrimed
	done = FALSE;						// TRUE = routine succesful, else FALSE
	overheat_count = 0;

	LCDSendCmd(CLR_DISP);						//	Display the number of pulses and delay user entered
	LCDSendStr(RSLMsg3);
	Display_Mode();								// Display the current mode
	Display_Pulse_Count(pulses);				// Display the new number of pulses remaining

	while(take_xrays == TRUE){					// loop until system is done taking xrays
		
		xray_confirmed = Trigger_Single_X_Ray();					// if true a confirmation pulse was detected, USER_CANCEl = cancelled, else false		

		if(xray_confirmed == TRUE){ 								// if a confirmation pulse was detected, then keep X_Ray_Type
			pulses--;												// reduce pulse count
			Display_Pulse_Count(pulses);							// Display the new number of pulses remaining
		} else if(xray_confirmed == USER_CANCEL){					// if the user cancelled while looking for confirmation pulse					
			cancelled = TRUE;
		} else if(xray_confirmed == FALSE){
			Feedback_Error();										// if all types have been tried, give error	
		}			
		
		if(pulses >= 1 && cancelled == FALSE) {						// if there are more xrays needed and the user didnt cancel
			cancelled = Xray_Delay(numdelay);						// Between xrays delay, returns TRUE if user presses a button
		} else if (pulses < 1){
			take_xrays = FALSE;
			done = TRUE;											// if pulse count reaches zero the routine is done 
		}
		
		if(cancelled == TRUE){ 										// if at any point the user cancelled the routine, stop the loop
			Display_Cancelled(pulses);								// Display that cancel was activated
			take_xrays = FALSE;										// stop  loop
			done = FALSE;
		}
	}

	return(done);
}


/*****************************************************************************
* Function Name: Fire_Modes_1and3
* 
* Description:  Fire modes 1 and 3 execute a set number of continous xrays, breaking
*				after each group of 50 to let the system cool down before continuing. 
*				In mode 1 the cool down period is 10s, in mode 3 it is 5s.  The pulses
*				are triggered continously by holding down the trigger and counting conformation
*				pulses on the feedback pin with the comparator.  The system releases the
*				trigger when the second to last xray conformation is detected, because by
*				the time a confirmation is detected the next xray has already been triggered.
*				NOTE: You can not get a continous set of xrays by pressing and releasing the 
*					the trigger, because the xray system will not trigger again for 800ms.
*					Also, if the trigger is just held down and no cool down period is used
*					the xray system will automaticaly stop after 99 xrays.
*
* Arguments: pulses, the number of xrays to take
*
* Returns: done, TRUE = finished, FALSE = didnt finish
*
*******************************************************************************/
char Fire_Modes_1and3(int pulses)
{
	unsigned char take_xrays, cancelled, xray_confirmed,done;
	unsigned int overheat_count;
				
	// initialize local vars
	take_xrays = FALSE;					// TRUE = take more xrays, FALSE = stop			
	cancelled = FALSE;					// TRUE = user requests stop, FALSE = continue
	xray_confirmed = FALSE;				// TRUE = xray feeback confirmed, FALSE = no xray confrimed
	done = FALSE;						// TRUE = routine succesful, else FALSE
	overheat_count = 0;

	if(pulses > 1){						// Error check: if the pulse count is greater then zero take xrays	
		take_xrays = TRUE;	
		Hold_X_Ray_Trigger();			// Trigger xrays
	}else if(pulses == 1){
		take_xrays = FALSE;
		xray_confirmed = Trigger_Single_X_Ray();	// if true a confirmation pulse was detected, USER_CANCEl = cancelled, else false		
		
		if(xray_confirmed == TRUE){ 								// if a confirmation pulse was detected
			pulses--;												// reduce pulse count
			Display_Pulse_Count(pulses);							// Display the new number of pulses remaining
		} else if(xray_confirmed == USER_CANCEL){					// if the user cancelled while looking for confirmation pulse					
			cancelled = TRUE;
		} else if(xray_confirmed == FALSE){
			Feedback_Error();										// if all types have been tried, give error	
		}
	}

	while(take_xrays == TRUE){			// loop until system is done taking xrays
		
		xray_confirmed = Confirm_X_Ray();

		if(xray_confirmed == TRUE){											// if a conformation pulse is detecteded
			pulses--;														// reduce pulse count
			overheat_count++;												// increment overheat count
			xray_confirmed = FALSE;											// reset flag
			Display_Pulse_Count(pulses);									// Display the number of pulses remaining
			
			if((pulses <= 1)||(overheat_count >= 49)){						// Stop taking xrays one early, there should be one extra generated
				Release_X_Ray_Trigger();									// Stop xrays
				pulses--;													// reduce pulse count
				overheat_count++;											// increment overheat count
				Display_Pulse_Count(pulses);								// Display the number of pulses remaining			

				if(pulses < 1){			 									// Stop taking xrays 
					take_xrays = FALSE;										// stop loop				
				}else if(overheat_count >= 49){								// Cool down, Stop taking xrays
					cancelled = Overheat_Delay();							// Delay system, monitor for user cancel, if user cancel set flag
					overheat_count = 0;										// Reset counter
					if(cancelled == FALSE && (pulses > 1)){ 
						Hold_X_Ray_Trigger();								// if the user didnt cancel operations, restart xrays
					} else if (cancelled == FALSE && (pulses == 1)){		// This fixes the problem with a pulse count of any incremntent of 50+1.
						xray_confirmed = Trigger_Single_X_Ray();			// if true a confirmation pulse was detected, USER_CANCEl = cancelled, else false								
						if(xray_confirmed == TRUE){ 						// if a confirmation pulse was detected
							pulses--;										// reduce pulse count
							overheat_count++;								// increment overheat count
							Display_Pulse_Count(pulses);					// Display the new number of pulses remaining
						} else if(xray_confirmed == USER_CANCEL){			// if the user cancelled while looking for confirmation pulse					
							cancelled = TRUE;	
						} else if(xray_confirmed == FALSE){
							//Feedback_Error();								// if all types have been tried, give error	
							X_Ray_Type = 0;									// if trigger fails, reset type and try again
							xray_confirmed = Trigger_Single_X_Ray();		// retry single trigger with learning mode
							if(xray_confirmed == FALSE) Feedback_Error();	// if still no pulse, trigger error
						}
						take_xrays = FALSE;									// stop loop	
					}
				}
			}

		} else if(xray_confirmed == USER_CANCEL){				// if the user cancelled during the confrimation detection
			cancelled = TRUE;									// set the flag for cancelling
		} else if(xray_confirmed == FALSE){  					// if Confirm_X_Ray(); returns false then it timeout and there was an error
			Release_X_Ray_Trigger();									// Stop xrays
			Feedback_Error();
		}	

		if(cancelled == TRUE){
			Release_X_Ray_Trigger();							// Stop xrays			
			take_xrays = FALSE;									// stop loop
		}			
	}

	if(cancelled == TRUE){
		Display_Cancelled(pulses);								// Display that cancel was activated
		done = FALSE;
	}else{
		done = TRUE;
	}

	return(done);

}



/*****************************************************************************
* Function Name: Check_Backlight()
* 
* Description:  input button var from routine and if it is button push the backlight turns
*					on and reset the timer,  else it checks the timer and turns the 
*					backlight off when the timer is zero. 
*				
*
* Arguments: button, if  a button was pushed, then reset timeout
*
* Returns: Nothing
*
*******************************************************************************/
void Check_Backlight(unsigned char button)
{
	if(button != BUTTON_NO){
		BLITE = ON;											//	Turn on the back lite
		Backlight_Timer = 2000;								// 10 secs
	} else if (Backlight_Timer <= 0){
		BLITE = OFF;										//	Turn on the back lite
	}
}

/*****************************************************************************
* Function Name: Fire_XRays
* 
* Description:  Fire modes 1 and 3 execute a set number of continous xrays, breaking
*				after each group of 50 to let the system cool down before continuing. 
*				In mode 1 the cool down period is 10s, in mode 3 it is 5s.  The pulses
*				are triggered continously by holding down the trigger and counting conformation
*				pulses on the feedback pin with the comparator.  The system releases the
*				trigger when the second to last xray conformation is detected, because by
*				the time a confirmation is detected the next xray has already been triggered.
*				NOTE: You can not get a continous set of xrays by pressing and releasing the 
*					the trigger, because the xray system will not trigger again for 800ms.
*					Also, if the trigger is just held down and no cool down period is used
*					the xray system will automaticaly stop after 99 xrays.
*
* Arguments: pulses, the number of xrays
*
* Returns: Nothing
*
*******************************************************************************/
void Fire_XRays(int pulses)
{
	unsigned char done, cancelled;

	done = FALSE;									// TRUE if Xrays fired correctly, else FALSE
	cancelled = FALSE;								// TRUE if user cancelled, else FALSE

	TRISB 	= 0xEF;     							// All as input, except LED pin.  Turn LED on to test.  
	
	//Display_Step_Back();							// Display "Please step back"
							
	//Pre_Fire_Beeping();							// Beep for 4 seconds	 	                
	
	//Display_Countdown();							// Dipslay a countdown timer for the last 5 secs	

	cancelled = Pre_Fire_Countdown();				// Display prefire countdown

	if(cancelled == FALSE){
		
		LCDSendCmd(CLR_DISP);							// Clear display
		LCDSendStr(RSLMsg3);							//	Display "PULSES"
		Display_Pulse_Count(pulses);					// Display the number of pulses remaining
		Display_Mode();									// Display the current mode
	
		Init_TMR1();									// Initialize timer1 and interrupts.
	
		if(X_Ray_Mode == DELAYED){
			done = Fire_Modes_2(pulses); 				// if delay mode
		} else if (X_Ray_Mode == SAFE || X_Ray_Mode == FAST){
			done = Fire_Modes_1and3(pulses);			// if mode 1 or 3	
		}
	
		if(done == TRUE)								// Display "DONE" after finishing fire
		{
			Display_Done(pulses);
		}
	} else {
		Display_Cancelled(pulses);						// Display that cancel was activated
	}	
}	

/*****************************************************************************
* Function Name: Get_Pulse_Count
* 
* Description:  monitor button pushes to increase or decease xray count,
*					Global_Pulse_Count is temporary until replace with locals
*					increase/decrease amount is based on cursor position
*
* Arguments: None
*
* Returns: proceed, TRUE =  enter was pressed, FALSE = cancel was pushed
*
*******************************************************************************/
char Get_Pulse_Count(void)
{
	unsigned char proceed, loop, pos_cursor, button;

	proceed = FALSE;													// Initialize flags
	loop = TRUE;
	pos_cursor = 0;														// set the starting location of the cursor

	LCDSendCmd(CLR_DISP);												// Init the screen
	LCDSendStr(RSLMsg3);												//	Display PULSES 0000 default
	Display_Pulse_Count(Global_Pulse_Count);							// Display the number of pulses 
	Display_Mode();
	LCDSendCmd(pos_cursor +  DD_RAM_ADDRP1);							// cursor postion is offest plus starting postion

	while(loop == TRUE)
	{
		button = Scan_Buttons();										// check for a button push

		Check_Backlight(button);										// insert function for backlight, on when button push and reset timer counter, check timer counter and turn off
		
		if(button == BUTTON_UP){
			Beep(150,61);
			if(Global_Pulse_Count <= 2999 && pos_cursor == 0){ 			// Error check before incrementing, pos=0 is thousands place
				Global_Pulse_Count += 1000;							
			} else if(Global_Pulse_Count <= 3899 && pos_cursor == 1){ 	// Error check before incrementing, pos=1 is hundreds place
				Global_Pulse_Count += 100;
			} else if(Global_Pulse_Count <= 3989 && pos_cursor == 2){ 	// Error check before incrementing, pos=2 is tens place
				Global_Pulse_Count += 10;
			} else if(Global_Pulse_Count <= 3998 && pos_cursor == 3){ 	// Error check before incrementing, pos=3 is ones place
				Global_Pulse_Count++;
			}
			Display_Pulse_Count(Global_Pulse_Count);					// Display the number of pulses 
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRP1);					// cursor postion is offest plus starting postion
		} else if(button == BUTTON_DOWN){
			Beep(150,61);
			if(Global_Pulse_Count >= 1000 && pos_cursor == 0){ 			// Error check before decrementing, pos=0 is thousands place
				Global_Pulse_Count -= 1000;							
			} else if(Global_Pulse_Count >= 100 && pos_cursor == 1){ 	// Error check before decrementing, pos=1 is hundreds place
				Global_Pulse_Count -= 100;
			} else if(Global_Pulse_Count >= 10 && pos_cursor == 2){ 	// Error check before decrementing, pos=2 is tens place
				Global_Pulse_Count -= 10;
			} else if(Global_Pulse_Count >= 1 && pos_cursor == 3){ 		// Error check before decrementing, pos=3 is ones place
				Global_Pulse_Count--;
			}
			Display_Pulse_Count(Global_Pulse_Count);					// Display the number of pulses 
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRP1);					// cursor postion is offest plus starting postion
		} else if(button == BUTTON_LEFT){
			Beep(150,61);
			if(pos_cursor > 0) pos_cursor--;							// Error check before decrementing			
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRP1);					// cursor postion is offest plus starting postion
		} else if(button == BUTTON_RIGHT){
			Beep(150,61);
			if(pos_cursor < 3) pos_cursor++;							// Error check before incrementing			
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRP1);					// cursor postion is offest plus starting postion
		} else if(button == BUTTON_ENTER){								
			Beep(150,61);
			proceed = TRUE;												// ENTER button was pressed
			loop = FALSE;
		} else if(button == BUTTON_CANCEL){
			Beep(150,61);
			proceed = FALSE;											// CANCEL button was pressed
			loop = FALSE;
		}
	}

	return(proceed);
}

/*****************************************************************************
* Function Name: Select_Mode
* 
* Description:  monitor button pushes to change the mode of operation, setting is stored in X_Ray_Mode
*
* Arguments: None
*
* Returns: None
*
*******************************************************************************/
void Select_Mode(void)
{
	unsigned char loop, button, pos_cursor;
													
	loop = TRUE;														// Initialize flag
	pos_cursor = 0;														// set the starting location of the cursor

	LCDSendCmd(CLR_DISP);												// Init Screen, with "Select:" and the current mode and a cursor if needed
	Display_Select();
	Display_Mode();
	if(X_Ray_Mode != DELAYED) LCDSendCmd(DD_RAM_ADDRP4+9);				// Move cursor out of the screen for easy to see
	else LCDSendCmd(pos_cursor +  DD_RAM_ADDRD1);						// cursor postion is offest plus starting postion	

	while(loop == TRUE)
	{
		button = Scan_Buttons();										// check for a button push

		Check_Backlight(button);										// insert function for backlight, on when bu333tton push and reset timer counter, check timer counter and turn off
		
		if(button == BUTTON_UP && X_Ray_Mode == DELAYED){
			Beep(150,61);
			if(numdelay <= 899 && pos_cursor == 0){ 					// Error Check, delay cant be more then 9.99 sec, else add a 1sec
				numdelay += 100;
			} else if(numdelay <= 989 && pos_cursor == 2){ 				// Error Check, delay cant be more then 9.99 sec, else add a 100ms
				numdelay += 10;
			} else if(numdelay <= 998 && pos_cursor == 3){ 				// Error Check, delay cant be more then 9.99 sec, else add a 10ms
				numdelay++;
			} 

			Display_Delay(numdelay);									// Display the number of pulses remaining
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRD1);					// cursor postion is offest plus starting postion of delay setting
		} else if(button == BUTTON_DOWN && X_Ray_Mode == DELAYED){
			Beep(150,61);
			if(numdelay >= 200 && pos_cursor == 0){ 					// Error Check, delay cant be lessthen 1sec, else subtract a second
				numdelay -= 100;
			} else if(numdelay >= 110 && pos_cursor == 2){ 				// Error Check, delay cant be lessthen 1sec, else subtract a 100 ms
				numdelay -= 10;
			} else if(numdelay >= 101 && pos_cursor == 3){ 				// Error Check, delay cant be lessthen 1sec, else subtract a 10 ms
				numdelay--;
			} 

			Display_Delay(numdelay);									// Display the number of pulses remaining
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRD1);					// cursor postion is offest plus starting postion of delay setting
		} else if(button == BUTTON_LEFT && X_Ray_Mode == DELAYED){
			Beep(150,61);
			if(pos_cursor > 0) pos_cursor--;							// Error check before decrementing		
				
			if(pos_cursor == 1) pos_cursor = 0;							// pos_cursor one is the '.'
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRD1);					// cursor postion is offest plus starting postion
		} else if(button == BUTTON_RIGHT && X_Ray_Mode == DELAYED){
			Beep(150,61);
			if(pos_cursor < 3) pos_cursor++;							// Error check before incrementing	

			if(pos_cursor == 1) pos_cursor = 2;							// pos_cursor one is the '.'		
			LCDSendCmd(pos_cursor +  DD_RAM_ADDRD1);					// cursor postion is offest plus starting postion
		} else if(button == BUTTON_ENTER){		
			Beep(150,61);									
			loop = FALSE;												// ENTER button was pressed
		} else if(button == BUTTON_CANCEL){
			Beep(150,61);
			X_Ray_Mode++;												// Goto next mode
			if(X_Ray_Mode>2) X_Ray_Mode = 0;							// there are only three modes, wrap around
			
			Display_Mode();
			if(X_Ray_Mode != DELAYED) LCDSendCmd(DD_RAM_ADDRP4+9);		// Move cursor out of the screen for easy to see
			else LCDSendCmd(pos_cursor +  DD_RAM_ADDRD1);				// cursor postion is offest plus starting postion	
			
		}
	}
}

int main(void) 
{	
	unsigned char pulse_set, goto_fire, pulse_loop, fire_loop, button, button2;
	int a=0,b=0, temp;

	TMR1ON=0;
	TMR1CS=0;
	T1CKPS0=1;
	T1CKPS1=1;
	TMR1H = 0xF3;			// 5ms is 65535 - (5ms/1.6us) = 62410(0xF3CA)
	TMR1L = 0xCA;

	start();
	Digital886();	

	Init_Comparator();						//Set up the comparator
	TRISC 	= 0x80;                     	//	PORTC: All output except bit 7 (RXD of the DB9)
	TRISA4=0;								//	Set RA4 as output
	BLITE=ON;								//	Turn on the back lite as the beginning
	DisplaySplashScreen();
	
	INTCON 	= 0x00;							//	Disable interrupt
	
	unsigned char m=29;						//	Delay when power up, with m=29 delay 2.1702 secs

	while(m--)								//	Delay when power up 
	{	
		Beep(150,61);  
	}
	
	LCDSendCmd(CLR_DISP);
	LCDSendStr(RSLMsg3);					//	Display PULSES 0000 default
	BLITE = OFF;							//  Turn off back lite, wait of button pressed

	Display_Pulse_Count(Global_Pulse_Count); // Display the number of pulses remaining
	LCDSendCmd(DD_RAM_ADDR2);
	
	LCDSendStr(RSLMsg14);  					//	Display FAST SAFE MODE	
	
	LCDSendCmd(DD_RAM_ADDRP1);
	LCDSendCmd(CUR_ON_BLINK);     			//	Blinking cursor @ first digit of PULSES for user to enter in

	//	PORTB
	INTCON 	= 0x00;							//	Disable interrupt
	OPTION 	= 0x80;							//	PORTB pull-ups are disabled

	//Clear variable
	B1=B2=B3=B4=B5=B6=0;
	//loop forever - scan the buttons

	Init_TMR1();							// Initialize timer1 and interrupts.

	Global_Pulse_Count = 0;					// Starting number of pulses and min
	numdelay = 100; 						// Starting length of delay and min
	pulse_loop = TRUE;

	while(1) 
	{	

		if(pulse_loop == FALSE) {		
			Select_Mode();												// First menu, the user selects the mode of operation, not displayed on 
			pulse_loop = TRUE;
		}	
													

		while(pulse_loop == TRUE)
		{
			pulse_set = Get_Pulse_Count();								// Second menu, the user selects the number of pulses, returns TRUE if user wants to go forward, else FALSE and return to Select_Mode
	
			if(pulse_set == FALSE){										// return to select mode, exit loop
				pulse_loop = FALSE;
			}else if(pulse_set == TRUE){								// move foward with firing
				LCDSendCmd(CLR_DISP);									// Init the screen
				Display_Estimated_Time();
				//Display_Prepare_to_Fire();								// Pre count down warning

				fire_loop = TRUE;
				goto_fire = FALSE;

				a=b=0;
				while(fire_loop == TRUE)								// this loop waits for user to confirm going forward and will then reshow the last settings when done so they can do it again, or they can hit cancel and go back to setting the pulse count
				{				
					a++;
					if ( a%500 == 0 ){
						if ( b == 0 ){
							b= 1;
							Display_Estimated_Time();
						}
						else{
							b=0;
							Display_Prepare_to_Fire();
						}
					}
										
						
					button = Scan_Buttons();							// check for a button push
	
					Check_Backlight(BUTTON_UP);							// insert function for backlight, on when button
					
					if(button == BUTTON_ENTER){																
						goto_fire = TRUE;								// ENTER button was pressed
					} else if(button == BUTTON_CANCEL){
						fire_loop = FALSE;
					}
					
					if(goto_fire == TRUE){
						Fire_XRays(Global_Pulse_Count);					// Make xrays
						fire_loop = FALSE;								// reset flags
						goto_fire = FALSE;
					}
				}
			}		
		}
	}	                                        
}