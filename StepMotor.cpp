/** \file StepMotor.cpp
 *	\brief 
 */

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
#include "StepMotor.h"
// ---------- EXTERNAL MODULE INCLUDE ------------------------------------------------------------ //
// N/A
// ---------- PROGRAMMING DEFINITION INCLUDE ----------------------------------------------------- //
// N/A
// ---------- EXTERN OBJECT ---------------------------------------------------------------------- //
// N/A
// ---------- PUBLIC INTERFACE METHOD ------------------------------------------------------------ //
// N/A
// ---------- PUBLIC METHOD (FUNCTION PROTOTYPE) ------------------------------------------------- //
// N/A
// ---------- PUBLIC DATA ----------------------------------------------------------------------- //
// N/A
// ---------- PRIVATE METHOD (FUNCTION PROTOTYPE) ----------------------------------------------- //
void TimerStart();
void TimerStop();
void UpdateTimer(uint16_t value);

// ---------- PRIVATE DATA ---------------------------------------------------------------------- //
#ifndef MOTION_IMPLEMENT_ISR
Motor::StepMotor stepMotor;
#endif
// ---------- PRIVATE PROGRAMMING DEFINE -------------------------------------------------------- //
// N/A
// ---------- PRIVATE MACRO DEFINITION ---------------------------------------------------------- //
// N/A
// ---------- SOURCE FILE IMPLEMENTATION -------------------------------------------------------- //
namespace Motor {

StepMotor::StepMotor() {
	// TODO Auto-generated constructor stub

}

StepMotor::~StepMotor() {
	// TODO Auto-generated destructor stub
}

//=========== Public Method ======================================================================//
uint8_t StepMotor::Init(uint8_t dirPin, MotorDirLogic_t dirType) {
	// Timer/Counter 1 in mode 4 CTC (Not running).
	// Timer/Counter 1 Output Compare A Match Interrupt enable.
	cli();
	TCCR1A |= //(1 << COM1A1) | (1 << COM1A0) |
			(1 << COM1B1) | (1 << COM1B0) | (1 << WGM10);
	TCCR1B = 1 << WGM13; // | 2;
	TIMSK1 = (1 << TOIE1) | (1 << OCIE1B);
	OCR1A = 0;
	OCR1B = 0;

	sei();
	this->move = MOVE_POSITION;
	this->dirPin = dirPin;
	this->dirState = dirType;
	this->isRunning = false;
	pinMode(5, OUTPUT);
}

uint8_t StepMotor::SetDirPin(uint8_t  dirPin) {
	this->dirPin = dirPin;
	pinMode(dirPin, OUTPUT);
	return this->dirPin;
}

uint8_t StepMotor::SetDirLogic(MotorDirLogic_t dirType) {
	this->dirState = dirType;
	return this->dirState;
}

uint32_t StepMotor::MoveAbs(uint32_t absolute) {
//	Serial.println("Move abs");
//	Serial.println("currentStep : " + String(this->currentStep, DEC) );
//	Serial.println("absolute : " + String(absolute, DEC) );
	return (this->Move(absolute - this->currentStep));
}

uint32_t StepMotor::MoveInc(uint32_t relative) {

	return (this->Move(relative));
}

uint32_t StepMotor::Move(int32_t step, uint32_t speed, uint32_t acc, uint32_t dec) {
	int32_t distance;
	uint32_t frq;

	// Set speed,acc,decel
	this->Setspeed(speed);
	this->SetAccelerator(acc);
	this->SetDeccelerator(dec);

	return (this->Move(step));

}

uint32_t StepMotor::Move(int32_t step) {
	int32_t distance;
	if (this->isRunning) {
		return 0;
	}
	this->targetStep += step;
	this->dir = this->targetStep > this->currentStep ? DIR_CW : DIR_CCW;

	// Write dir pin
	if (this->dir == DIR_CW) {
		digitalWrite(this->dirPin, this->dirState == CW_LOW ? LOW : HIGH);
	}
	else {
		digitalWrite(this->dirPin, this->dirState == CW_LOW ? HIGH : LOW);
	}

	// get step to go
	distance = this->targetStep - currentStep;
	distance = abs(distance);

	// Calculate initial value
	this->n = 0;
	this->minDelay = this->calMinDelay();
	this->c0 = this->calC0();
	this->cn = this->c0;
	//this->cn = this->calCn(this->n);
	this->stepToSpeed = this->calStepToSpeed();
	this->stepToStop = this->calStepToStop();
	uint32_t acc_lim = this->calAccLimit(distance);
	int32_t dec_val = 0;
	this->stepToDec = distance - this->stepToStop;
	if (acc_lim <=  this->stepToSpeed){
		dec_val = acc_lim - distance;
		this->stepToStop = (uint32_t)(-dec_val);
		this->stepToDec = acc_lim;
	}
	else {
		dec_val = -this->calStepToStop();
	}

	//Serial.println("Start Move");
	//	Serial.println("Step : " + String(step, DEC) );
//		Serial.println("Min Delay : " + String(this->minDelay, DEC) );
//		Serial.println("stepToSpeed : " + String(this->stepToSpeed, DEC) );
//		Serial.println("stepToStop : " + String(this->stepToStop, DEC) );
//		Serial.println("stepToDec : " + String(this->stepToDec, DEC) );
//		Serial.println("acc_lim : " + String(acc_lim, DEC) );
//		Serial.println("dec_val : " + String(dec_val, DEC) );
	if (this->stepToDec < 0) {
		// error
		// cann't perform
		//Serial.println("can't Move");
		return 0;
	}

	// Change state
	this->state = StepMotor::ACC;
	this->rest = 0;
	this->accelCount = 0;
//	Serial.println("C0 : " + String(this->c0, DEC) );
	UpdateTimer((uint16_t)this->c0);
	// Calculate next cn
	++this->n;
	this->cn = this->calCn(this->n);
//	Serial.println("Cn : " + String(this->cn, DEC) );
	this->isRunning = true;
	TimerStart();

	return this->currentStep;

}

uint8_t StepMotor::Stop() {
	this->state = STOP;
	this->isRunning = false;
	TimerStop();
}

uint8_t StepMotor::Run(void) {
	uint32_t frq;
	int32_t _n;
	//Serial.println(String(this->n, DEC) +" : " + String(this->cn, DEC) );
	//	timer->UpdateFrequency(frq);
	UpdateTimer((uint16_t)this->cn);


	switch (this->state) {
	case STOP:
		// Do nothing
//		Serial.println("Stop " + String(this->currentStep, DEC));
		this->isRunning = false;

		TimerStop();
		break;
	case ACC:
		_n = ++this->n;
		this->accelCount++;
		this->cn = this->calCn(_n);
		if (_n >= this->stepToDec) {
			this->state = DECCEL;
			this->n = -this->stepToStop;
			this->accelCount = this->stepToStop;
		}
		else if (_n >= this->stepToSpeed) {
			if (this->move == MOVE_SPIN) {
				this->state = SPIN;
			} else {
				this->state = RUN;
			}
		}
	break;
	case RUN:
		_n = ++this->n;
		this->cn  = this->minDelay;

		// Go to Deceleration state.
		if (_n >= this->stepToDec) {
			this->state = DECCEL;
			this->n = -this->stepToStop;
			this->accelCount = this->stepToStop;
		}
		break;
	case DECCEL:
		_n = ++this->n;
		this->accelCount++;
		this->cn = this->calCn(_n);

		if (--this->stepToStop <= 0) {
			// Goto stop

			this->state = STOP;
		}
		break;
	case SPIN:
		this->cn = this->calCn(_n);
		break;
	}

//	frq = (uint32_t) (this->cn / this->fref);
//	timer->UpdateFrequency(frq);
	//UpdateTimer((uint16_t)this->cn);
	if (this->dir == DIR_CW) {
		this->currentStep++;
	}
	else {
		this->currentStep--;
	}

	return 1;

}

uint32_t StepMotor::SetCurrentPosition(uint32_t value) {
	this->Stop();
	this->currentStep = value;
	this->targetStep = value;
}

//=========== Private Method ======================================================================//
uint32_t StepMotor::calMinDelay(void) {
	uint32_t cMin;
	float cMinf;

	cMinf = STEP_ANGLE*TIMER_FREQ / this->maxSpeed;

	cMin = (uint32_t)cMinf;


	return cMin;

}

uint32_t StepMotor::calC0(void) {
	uint32_t _c0;
	float c0f;

	c0f = 0.676*TIMER_FREQ*sqrtf((2*STEP_ANGLE)/this->acc);

	_c0 = (uint32_t)c0f;

	return _c0;
}

uint32_t StepMotor::calCn(int32_t n) {
	uint32_t _cn;
	float cnf;
	float x,y;
	x = ((4 * n) + 1);
	y = (2 * this->cn + this->rest);
	this->rest = ((2 * (long) this->cn) + this->rest)
			% (4 * this->accelCount + 1);
	//Serial.println("cnf x: " + String(x, DEC) );
	//Serial.println("cnf y: " + String(y, DEC) );
	//Serial.println("cnf y/x: " + String(-(y/x), DEC) );
	//cnf = (int32_t)this->cn - ((2 * this->cn) / ((4 * n) + 1));
	cnf = (int32_t)this->cn - (y / x);
	//Serial.println("cnf : " + String(cnf, DEC) );
	_cn = (uint32_t)cnf;
	//Serial.println( String(n, DEC) + " _cn : " + String(_cn, DEC) );

	return _cn > this->minDelay ? _cn : this->minDelay;
}

uint32_t StepMotor::calSpeed(uint32_t c) {
	float spd;

	spd = (STEP_ANGLE * this->fref) / c;

	return (uint32_t) spd;
}

uint32_t StepMotor::calStepToSpeed(void) {
	float _n;

	_n = (this->speed * this->speed) / (2 * STEP_ANGLE * this->acc);

	return (uint32_t) _n;
}

uint32_t StepMotor::calAccLimit(uint32_t step) {
	float _n;

	_n = (step * this->dec) / (this->acc + this->dec);

	return (uint32_t) _n;
}

uint32_t StepMotor::calStepToStop(void) {
	float _n;

	_n = (this->speed * this->speed) / (2 * STEP_ANGLE * this->dec);

	return (uint32_t) _n;
}
} /* namespace Motor */

void TimerStart() {
	TCCR1B |= 2;
}

void TimerStop() {
	TCCR1B &= 0xF8;
}

void UpdateTimer(uint16_t value) {
	OCR1A = value;
	OCR1B = value / 2;
}

#ifndef MOTION_IMPLEMENT_ISR
ISR (TIMER1_OVF_vect) {
	//PORTD ^= 1 << 6;
	//PORTD ^= 1 << 7;
	//MOTOR::MotorControl::Callback (&motionControl);
	stepMotor.Run();

}

ISR (TIMER1_COMPB_vect) {
	PORTD ^= 1 << 5;
	//MotorControl::CallBack(&motionControl);
}
#endif
// ---------- END OF SOURCE FILE IMPLEMENTATION ------------------------------------------------- //
