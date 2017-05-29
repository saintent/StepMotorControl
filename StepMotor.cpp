/** \file StepMotor.cpp
 *	\brief 
 */

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
#include "arm_math.h"
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
MOTOR::StepMotor motionControl;
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
uint8_t StepMotor::Init() {
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
}

uint32_t StepMotor::Move(int32_t step, uint32_t speed, uint32_t acc, uint32_t dec) {
	int32_t distance;
	uint32_t frq;

	// Set speed,acc,decel
	this->Setspeed(speed);
	this->SetAccelerator(acc);
	this->SetDeccelerator(dec);

	this->targetStep += step;
	this->dir = this->targetStep > this->currentStep ? DIR_CW : DIR_CCW;

	// get step to go
	distance = this->targetStep - currentStep;
	distance = abs(distance);

	// Calculate initial value
	this->n = 0;
	this->minDelay = this->calMinDelay();
	this->c0 = this->calC0();
	//this->cn = this->calCn(this->n);
	this->stepToSpeed = this->calStepToSpeed();
	this->stepToStop = this->calStepToStop();
	this->stepToDec = distance - this->stepToStop;
	if (this->stepToDec < 0) {
		// error
		// cann't perform
		return 0;
	}


	// Change state
	this->state = StepMotor::ACC;
	//frq = (uint32_t) (this->c0 / this->fref);
	// Setup PWM and start
	//timer->UpdateFrequency(frq);
	UpdateTimer((uint16_t)this->c0);

	return this->currentStep;

}

uint8_t StepMotor::Run(void) {
	uint32_t frq;
	int32_t _n;

	switch (this->state) {
	case STOP:
		// Do nothing
		break;
	case ACC:
		_n = ++this->n;
		this->cn = this->calCn(_n);
		if (_n >= this->stepToDec) {
			this->state = DEC;
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
			this->state = DEC;
			this->n = -this->stepToStop;
		}
		break;
	case DEC:
		_n = ++this->n;
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
	UpdateTimer((uint16_t)this->cn);

	return 1;

}
//=========== Private Method ======================================================================//
uint32_t StepMotor::calMinDelay(void) {
	uint32_t cMin;
	float32_t cMinf;

	cMinf = STEP_ANGLE*TIMER_FREQ / this->rMaxSpeed;

	cMin = (uint32_t)cMinf;

	return cMin;

}

uint32_t StepMotor::calC0(void) {
	uint32_t _c0;
	float32_t c0f;

	c0f = 0.676*TIMER_FREQ*sqrtf((2*STEP_ANGLE)/this->rAcc);

	_c0 = (uint32_t)_c0;

	return _c0;
}

uint32_t StepMotor::calCn(int32_t n) {
	uint32_t _cn;
	float32_t cnf;

	cnf = this->cn - (2 * this->cn) / ((4 * n) + 1);

	_cn = (uint32_t)cnf;

	return _cn > this->minDelay ? _cn : this->minDelay;
}

uint32_t StepMotor::calSpeed(uint32_t c) {
	float32_t spd;

	spd = (STEP_ANGLE * this->fref) / c;

	return (uint32_t) spd;
}

uint32_t StepMotor::calStepToSpeed(void) {
	float32_t _n;

	_n = (this->rSpeed * this->rSpeed) / (2 * PI * this->rAcc);

	return (uint32_t) _n;
}

uint32_t StepMotor::calAccLimit(uint32_t step) {
	float32_t _n;

	_n = (step * this->rDec) / (this->rAcc + this->rDec);

	return (uint32_t) _n;
}

uint32_t StepMotor::calStepToStop(void) {
	float32_t _n;

	_n = (this->rSpeed * this->rSpeed) / (2 * PI * this->rDec);

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
	PORTD ^= 1 << 7;
	//MOTOR::MotorControl::Callback (&motionControl);
	motionControl.Run();

}

ISR (TIMER1_COMPB_vect) {
	PORTD ^= 1 << 6;
	//MotorControl::CallBack(&motionControl);
}
#endif
// ---------- END OF SOURCE FILE IMPLEMENTATION ------------------------------------------------- //
