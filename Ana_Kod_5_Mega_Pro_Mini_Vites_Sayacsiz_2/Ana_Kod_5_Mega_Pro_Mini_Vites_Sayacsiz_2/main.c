
#ifndef F_CPU					
#define F_CPU 16000000UL		// define it now as 16 MHz unsigned long
#endif

#include <avr/io.h>
#include "millis.h"
#include <util/delay.h>
#include <avr/interrupt.h>

#define BRAKE_LIGHT_INIT_PORTS()		do{DDRB &= ~(1<<PINB3); PORTB |= (1<<PINB3); DDRA |= (1<<PINA4);}while(0)

#define SWITCH_PRESSED_PB3 !(PINB & (1<<PINB3))

// PID Define
// 
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1
// 

// Definitions for Motor Driver
// 
#define Motor_Speed_Pin()				do{DDRB |= (1<<PINB7);}while(0)

#define Motor_Speed_Pin_Signal()		do{PORTB |= ( 1 << PINB7);}while(0)
#define Motor_Speed_Pin_Non_Signal()	do{PORTB &= ~( 1 << PINB7);}while(0)
// 

// Definitions for ECU
// 
#define EcuThrottle_Init_Ports()					do{DDRB |= (1<<PINB4);}while(0)
// 

// Definitions for the analogRead Function
// 
#define BIT_IS_SET(byte, bit) (byte & (1 << bit))
#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))
// 

// Define variable for Brake Light
// 
uint8_t Brake_Light_Durum = 0;
// 

// PID Control Variables
// 
double dispKp;				//  With temporary variable for display purpose, 
double dispKi;				//  we will keep setting parameters in user entered format
double dispKd;

double kp;                  //  Proportional Tuning Parameter
double ki;                  //  Integral Tuning Parameter
double kd;                  //  Derivative Tuning Parameter

int controllerDirection;
int pOn;


unsigned long lastTime;
double outputSum, lastInput;

unsigned long SampleTime;
double outMin, outMax;
int inAuto, pOnE;
// 

// TPS Max and Min configuration values
// 
int TPS_Min, TPS_Max, TPS;
// 

// Input and output values for the driver
// 
double Setpoint, Input, Output, myOutput;
// 

// Configuration values for the motor driver
// 
double consKp = 8 , consKi = 0.000001 , consKd = 1.2 ;
// 

// Brake Light Function defines
// 
void BrakeLight_Init_Ports();
void Brake_Light();
// 

// Function definitions for PID
// 
void PID(double Kp, double Ki, double Kd, int POn, int ControllerDirection);
int Compute();
void SetTunings(double Kp, double Ki, double Kd, int POn);
void SetSampleTime(int NewSampleTime);
void SetOutputLimits(double Min, double Max);
void SetMode(int Mode);
void Initalize();
void SetControllerDirection(int Direction);
// 

// analog R/W, map, ABS Defines functions
// 
int analogRead(int pin);
void analogWrite(uint8_t pin, int val);
long map(long x, long in_min, long in_max, long out_min, long out_max);
double ABS(double Number);
// 

// sub-functions to be used inside the main function
// 
void Motor_Surucu_Init_Ports();
void Throttle();
void Ecu_Throttle_Init_Ports();
// 

int main(void)
{
    init_millis(16000000UL);
    
    sei();
	
	BrakeLight_Init_Ports();
	Motor_Surucu_Init_Ports();
	
	
    while (1) 
    {
		Brake_Light();
		Throttle();
		Ecu_Throttle_Init_Ports();
    }
}

void BrakeLight_Init_Ports()
{
	BRAKE_LIGHT_INIT_PORTS();
}

void Motor_Surucu_Init_Ports()
{
	
	Motor_Speed_Pin();
	
	// Application of Max and Min configuration value
	
	Motor_Speed_Pin_Signal();
	
	_delay_ms(1000);
	
	TPS_Max = analogRead(2);
	
	Motor_Speed_Pin_Non_Signal();
	
	_delay_ms(1000);
	
	TPS_Min = analogRead(2);
	
	
	TPS = map(analogRead(2), TPS_Min, TPS_Max, 0, 255);
	
	Input = TPS;
	
	Setpoint = map(analogRead(0), 18, 102, 0,255);
	
	PID(consKp, consKi, consKd, P_ON_E, DIRECT);
	
	SetMode(AUTOMATIC);
}

void Ecu_Throttle_Init_Ports()
{
	EcuThrottle_Init_Ports();
}

void Brake_Light()
{
	if(SWITCH_PRESSED_PB3)
	{
		Brake_Light_Durum = 1;
	}
	else
	{
		Brake_Light_Durum = 0;
	}
	
	if(Brake_Light_Durum == 1)
	{
		PORTA |= (1 << PINA4);
	}
	else
	{
		PORTA &= ~(1 << PINA4);
	}
}

void Throttle()
{
	TPS = map(analogRead(2), TPS_Min, TPS_Max, 0, 255);
	
	Input = TPS;
	
	
	
	Setpoint = map(analogRead(0), 18, 102, 0, 255);
	
	PID(consKp, consKi, consKd, P_ON_E, DIRECT);
	
	SetTunings(consKp, consKi, consKd, pOn);
	
	PID(consKp, consKi, consKd, P_ON_E, DIRECT);
	
	Compute();
	
	
	analogWrite(13, Output);
}

void PID(double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
	
	SetOutputLimits(0,255);
	
	SampleTime = 100;
	
	SetControllerDirection(ControllerDirection);
	SetTunings(Kp, Ki, Kd, POn);
	
	lastTime = millis()-SampleTime;
	
}

int Compute()
{
	if(!inAuto) return 0;
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	if(timeChange>=SampleTime)
	{
		/*Compute all the working error variables*/
		double input = Input;
		double error = Setpoint - input;
		double dInput = (input - lastInput);
		outputSum+= (ki * error);

		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if(!pOnE) outputSum-= kp * dInput;

		if(outputSum > outMax) outputSum= outMax;
		else if(outputSum < outMin) outputSum= outMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		double output;
		if(pOnE) output = kp * error;
		else output = 0;

		/*Compute Rest of PID Output*/
		output += outputSum - kd * dInput;

		if(output > outMax) output = outMax;
		else if(output < outMin) output = outMin;
		Output = output;

		/*Remember some variables for next time*/
		lastInput = input;
		lastTime = now;
		return 1;
	}
	else return 0;
}

void SetTunings(double Kp, double Ki, double Kd, int POn)
{
	pOn = POn;
	pOnE = POn == P_ON_E;

	dispKp = Kp; dispKi = Ki; dispKd = Kd;

	double SampleTimeInSec = ((double)SampleTime)/1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;

	if(controllerDirection ==REVERSE)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio  = (double)NewSampleTime
		/ (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}

void SetOutputLimits(double Min, double Max)
{
	if(Min >= Max)return;
	outMin = Min;
	outMax = Max;
	
	if(inAuto)
	{
		if(Output > outMax) Output = outMax;
		else if(Output < outMin) Output = outMin;

		if(outputSum > outMax) outputSum= outMax;
		else if(outputSum < outMin) outputSum= outMin;
	}
}

void SetMode(int Mode)
{
	int newAuto = (Mode == AUTOMATIC);
	if(newAuto && !inAuto)
	{  /*we just went from manual to auto*/
		Initalize();
	}
	inAuto = newAuto;
}

void Initalize()
{
	outputSum = Output;
	lastInput = Input;
	if(outputSum > outMax) outputSum = outMax;
	else if(outputSum < outMin) outputSum = outMin;
}

void SetControllerDirection(int Direction)
{
	if(inAuto && Direction !=controllerDirection)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = Direction;
}
// 

// analog R/W, map, ABS Functions
// 
int analogRead(int pin)
{
	uint8_t high;// low;
	
	if(pin==0)
	{
		ADMUX = 0b01100000;
	}
	else if(pin==2)
	{
		ADMUX = 0b01100010;
	}
	ADCSRA = 0b10000011;
	
	ADCSRB = 0b00000000;
	
	ADCSRA |= (1 << ADSC);
	
	while(BIT_IS_SET(ADCSRA, ADSC)) {}
	
	//low  = ADCL;
	high = ADCH;
	
	return (high);// >> 8) | low;
}

void analogWrite(uint8_t pin, int val)
{
	if(pin == 13)
	{
		TCCR0A = 0b10000001;
		TCCR0B = 0b00000001;
		OCR0A = val;
	}

}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double ABS(double Number)
{
	if(Number < 0)
	{
		Number = Number * (-1);
	}
	else
	{
		Number = Number * (1);
	}
	return Number;
	
}
// 