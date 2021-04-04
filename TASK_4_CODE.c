#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include <math.h>
#include<string.h>

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;


unsigned char sharp, distance, adc_reading;
unsigned int value;

// to store IR proximity sensor value
unsigned int lot = 0;
// array increment variables
unsigned int i = 0;
int j = -1;

// for loop variables
unsigned int k = 0;
unsigned int m = 0;

//slot number for parking
unsigned int rev_node = 0;
unsigned int slot = 0;


void timer5_init();


//variable for white line sensor
unsigned char lefts = 0;
unsigned char mids = 0;
unsigned char rghts = 0;

//variables for flag detection
unsigned char flag = 0;

//variables for IR proximity sensor
unsigned char lot_adc = 0;
unsigned char ref_adc = 0;



volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning


//LCD pin config
void lcd_port_config (void)
{
DDRC = DDRC | 0xF7;
PORTC = PORTC & 0x80;
}

//ADC pin configuration
void adc_pin_config (void)
{
DDRF = 0x00;
PORTF = 0xE0;
DDRK = 0x00;
PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
DDRA = DDRA | 0x0F;
PORTA = PORTA & 0xF0;
DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}


//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}


void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
cli(); //Clears the global interrupt
EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
cli(); //Clears the global interrupt
EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
ShaftCountLeft++;  //increment left shaft position count
}


void timer5_init()
{
TCCR5B = 0x00; //Stop
TCNT5H = 0xFF; //Counter higher 8-bit value to which OCR5xH value is compared with
TCNT5L = 0x01; //Counter lower 8-bit value to which OCR5xH value is compared with
OCR5AH = 0x00; //Output compare register high value for Left Motor
OCR5AL = 0xFF; //Output compare register low value for Left Motor
OCR5BH = 0x00; //Output compare register high value for Right Motor
OCR5BL = 0xFF; //Output compare register low value for Right Motor
OCR5CH = 0x00; //Output compare register high value for Motor C1
OCR5CL = 0xFF; //Output compare register low value for Motor C1
TCCR5A = 0xA9; /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
   For Overriding normal port functionality to OCRnA outputs.
   {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

TCCR5B = 0x0B; //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
ADCSRA = 0x00;
ADCSRB = 0x00; //MUX5 = 0
ADMUX = 0x20; //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
ACSR = 0x80;
ADCSRA = 0x86; //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
unsigned char a;
if(Ch>7)
{
ADCSRB = 0x08;
}
Ch = Ch & 0x07;  
ADMUX= 0x20| Ch;  
ADCSRA = ADCSRA | 0x40; //Set start conversion bit
while((ADCSRA&0x10)==0); //Wait for conversion to complete
a=ADCH;
ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
ADCSRB = 0x00;
return a;
}

//Sharp sensor distance Estimation function
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


void port_init()
{
lcd_port_config();
adc_pin_config();
motion_pin_config(); //robot motion pins config
left_encoder_pin_config(); //left encoder pin config
right_encoder_pin_config();
//right encoder pin config
}


void print_sensor(char row, char coloumn,unsigned char channel)
{

ADC_Value = ADC_Conversion(channel);
lcd_print(row, coloumn, ADC_Value, 3);
}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
OCR5AL = (unsigned char)left_motor;
OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; // removing upper nibbel for the protection
 PortARestore = PORTA; // reading the PORTA original status
 PortARestore &= 0xF0; // making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; // executing the command
}

void buzzer_pin_config (void)
{
 DDRJ = DDRC | 0x08; //Setting PORTC 3 as output
 PORTC = PORTC & 0xF7; //Setting PORTC 3 logic low to turnoff buzzer
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void forward (void)
{
  motion_set (0x06);
}

void back (void) //both wheels backward
{
motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
motion_set(0x05);
}

void back_left (void) //Left wheel backward, Right wheel forward
{
motion_set(0x01);
}

void back_right (void) //Left wheel backward, Right wheel forward
{
motion_set(0x08);
}


void soft_left (void) //Left wheel stationary, Right wheel forward
{
motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
motion_set(0x08);
}

//Function used for turning robot by specified degrees
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
//Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
float ReqdShaftCount = 0;
unsigned long int ReqdShaftCountInt = 0;

ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

ShaftCountRight = 0;
while(1)
{
if(ShaftCountRight > ReqdShaftCountInt)
{
break;
}
}
 //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
forward();
linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
back();
linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
left(); //Turn left
angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
right(); //Turn right
angle_rotate(Degrees);
}


void back_left_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
back_left(); //Turn left
angle_rotate(Degrees);
}



void back_right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
back_right(); //Turn right
angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_left(); //Turn soft left
Degrees=Degrees*2;
angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_right();  //Turn soft right
Degrees=Degrees*2;
angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_left_2(); //Turn reverse soft left
Degrees=Degrees*2;
angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_right_2();  //Turn reverse soft right
Degrees=Degrees*2;
angle_rotate(Degrees);
}


void stop (void)
{
  motion_set (0x00);
}


void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}



//Initialize the ports


//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz

void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
//Configure PORTK 5 pin for servo motor 4 operation
void servo4_pin_config (void)
{
 DDRH  = DDRH | 0x20;  //making PORTK 5 pin output
 PORTH = PORTH | 0x20; //setting PORTK 5 pin to logic 1
}

//Configure PORTK 6 pin for servo motor 2 operation
void servo5_pin_config (void)
{
 DDRH  = DDRH | 0x40;  //making PORTK 6 pin output
 PORTH = PORTH | 0x40; //setting PORTK 6 pin to logic 1
}

//Configure PORTK 7 pin for servo motor 3 operation
void servo6_pin_config (void)
{
 DDRH  = DDRH | 0x80;  //making PORTK 7 pin output
 PORTH = PORTH | 0x80; //setting PORTK 7 pin to logic 1
}

//Initialize the ports
void port_init1(void)
{
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
 servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
 servo4_pin_config(); //Configure PORTK 5 pin for servo motor 4 operation
 //servo5_pin_config(); //Configure PORTK 6 pin for servo motor 5 operation
 //servo6_pin_config(); //Configure PORTK 7 pin for servo motor 6 operation  
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03; //Output compare Register high value for servo 1
 OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
 OCR1BH = 0x03; //Output compare Register high value for servo 2
 OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
 OCR1CH = 0x03; //Output compare Register high value for servo 3
 OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
 ICR1H  = 0x03;
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
  For Overriding normal port functionality to OCRnA outputs.
 {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
void timer4_init(void)
{
 TCCR4B = 0x00; //stop
 TCNT4H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT4L = 0x01; //Counter low value to which OCR1xH value is to be compared with
 OCR4AH = 0x03; //Output compare Register high value for servo 4
 OCR4AL = 0xFF; //Output Compare Register low Value For servo 4
 OCR4BH = 0x01; //Output compare Register high value for servo 5
 OCR4BL = 0xFF; //Output Compare Register low Value For servo 5
 OCR4CH = 0x01; //Output compare Register high value for servo 6
 OCR4CL = 0xFF; //Output Compare Register low Value For servo 6
 ICR4H  = 0x03;
 ICR4L  = 0xFF;
 TCCR4A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
          For Overriding normal port functionality to OCRnA outputs.
          {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR4C = 0x00;
 TCCR4B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
//Function to initialize all the peripherals
//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}
//Function to rotate Servo 4 by a specified angle in the multiples of 1.86 degrees
void servo_4(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR4AH = 0x00;
 OCR4AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 5 by a specified angle in the multiples of 1.86 degrees
void servo_5(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR4BH = 0x00;
 OCR4BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 6 by a specified angle in the multiples of 1.86 degrees
void servo_6(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR4CH = 0x00;
 OCR4CL = (unsigned char) PositionServo;
}


//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03;
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
}
void servo_4_free (void) //makes servo 4 free rotating
{
 OCR4AH = 0x03;
 OCR4AL = 0xFF; //Servo 4 off
}

void servo_5_free (void) //makes servo 5 free rotating
{
 OCR4BH = 0x03;
 OCR4BL = 0xFF; //Servo 5 off
}

void servo_6_free (void) //makes servo 6 free rotating
{
 OCR4CH = 0x03;
 OCR4CL = 0xFF; //Servo 6 off
}

void init_devices1(void)
{
	cli(); //disable all interrupts
	port_init();
	timer1_init();
	timer4_init();
	sei(); //re-enable interrupts
}
void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
	init_devices1();
}

/**
* Servo function to pick objects
**/
void pick()
{

servo_2(100);
_delay_ms(1000);
servo_1(100);
_delay_ms(1000);

for(int k =100; k> 65;k--)

{
servo_2(k);
_delay_ms(100);


 }

 //servo_1(135);
 for( int j = 100; j < 135;j++)
 {
 servo_1(j);
 _delay_ms(300);
}

for(int i = 65; i < 90; i++)
{
servo_2(i);
_delay_ms(100);
}

}
/**
* Servo function to place objects
**/
void place()
{

for(int i = 100; i > 65; i--)
{
servo_2(i);
_delay_ms(100);
}

servo_1(100);
_delay_ms(1000);
for(int i = 65; i < 100; i++)
{
servo_2(i);
_delay_ms(100);
}
}
void left_wls()
{

if(!((lefts < 8) && (mids >9) && (rghts <8)))
{

left();
}
}
void right_wls()
{

if(!((lefts < 8) && (mids > 9) && (rghts <8)))
{

right();
}
}

void left_turn_wls(void)
{
while (!((ADC_Conversion(1)<9) && (ADC_Conversion(2) >10) && (ADC_Conversion(3)<9)))
{
left_degrees(40);
}

}

void right_turn_wls(void)
{
while (!((ADC_Conversion(1)<9) && (ADC_Conversion(2) >10) && (ADC_Conversion(3)<9)))
{
right_degrees(40);
}
}

int main()
{     port_init1();
 adc_pin_config ();
 adc_init();
 buzzer_pin_config();
 init_devices();
 lcd_set_4bit();
 lcd_init();
 lcd_cursor(1,3);
int count=0;  
int house=0;
 
while(1)  
{
house=1;
lcd_cursor(1,3);
print_sensor(1,2,1);
print_sensor(1,6,2);
print_sensor(1,10,3);

if( (ADC_Conversion(1)<7) && (ADC_Conversion(2)<7) && (ADC_Conversion(3)>6))
{
 soft_right_degrees(10);

}
if((ADC_Conversion(1)>6) && (ADC_Conversion(2)<7) && (ADC_Conversion(3)<7))
{
soft_left();
}
if ((ADC_Conversion(1)<7) && (ADC_Conversion(2)>6) && (ADC_Conversion(3)>6))
{
soft_right_degrees(10);
}
if ((ADC_Conversion(1)>6) && (ADC_Conversion(2)>6) && (ADC_Conversion(3)<7))
{
soft_left();
}
if ((ADC_Conversion(1)<7) && (ADC_Conversion(2) >6) && (ADC_Conversion(3)<7))
{
forward();
velocity(100, 100);
}
if ((ADC_Conversion(1)>6) && (ADC_Conversion(2) >6) && (ADC_Conversion(3)>6))
{ 
forward();
_delay_ms(500);
stop();
_delay_ms(100);
count++;
if(count==1||count==2)
{
while( !(ADC_Conversion(2)>6))
{
right();
}
}
if(count==5)
{
forward();
_delay_ms(100);
stop();
servo_1(130);
_delay_ms(500);
servo_1(35);
_delay_ms(500);
servo_3(15);
_delay_ms(500);
servo_1(130);
_delay_ms(500);
stop();
_delay_ms(2000);
right();
_delay_ms(2000);

}
if(count==7)
{
forward();
_delay_ms(100);
stop();
servo_2(15);
_delay_ms(500);
servo_2(105);
_delay_ms(500);
servo_4(50);
_delay_ms(500);
servo_2(15);
_delay_ms(500);


}
if(count==8||count==10)
{

stop();
_delay_ms(2000);
left();
_delay_ms(750);
}
if(count==12)
{
servo_2(30);
_delay_ms(1000);
servo_4(35);
_delay_ms(1000);
servo_4(15);
_delay_ms(500);
servo_2(15);
_delay_ms(1000);

}

if(count==14)
{
	stop();
	_delay_ms()
	servo_1(100);
	_delay_ms(1000);
	servo_3(35);
	_delay_ms(1000);
	servo_3(15);
	_delay_ms(500);
	servo_1(130);
	_delay_ms(1000);
}
}
