/** \file Motor.cpp
 *	\brief 
 */

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
//
#include "Motor.h"
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
// N/A
// ---------- PRIVATE DATA ---------------------------------------------------------------------- //
// N/A
// ---------- PRIVATE PROGRAMMING DEFINE -------------------------------------------------------- //
// N/A
// ---------- PRIVATE MACRO DEFINITION ---------------------------------------------------------- //
// N/A
// ---------- SOURCE FILE IMPLEMENTATION -------------------------------------------------------- //
namespace Motor {
//=========== Public Method ======================================================================//
Motor::Motor() {
	// TODO Auto-generated constructor stub

	this->acc = 1;

	this->dec = 1;

	this->speed = 0;

	this->maxSpeed = 10;

}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

void Motor::Setspeed(float speed) {
	this->speed = speed;
	this->maxSpeed = speed;
}

float Motor::GetSpeed(void) {
	return this->speed;
}

void Motor::SetAccelerator(float acc) {
	this->acc = acc;
}

float Motor::GetAccelerator(void) {
	return this->acc;
}

void Motor::SetDeccelerator(float dec) {
	this->dec = dec;
}

float Motor::GetDeccelerator(void) {
	return this->dec;
}
//=========== Private Method ======================================================================//

} /* namespace Motor */
// ---------- END OF SOURCE FILE IMPLEMENTATION ------------------------------------------------- //
