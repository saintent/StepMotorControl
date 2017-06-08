/** \file StepMotor.h
 *	\brief 
 */

#ifndef STEPMOTOR_H_
#define STEPMOTOR_H_

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
#include "stdint.h"
//#include "arm_math.h"
#include "Motor.h"
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "stdlib.h"
 #include "wiring.h"
#endif
// ---------- EXTERNAL MODULE INCLUDE ------------------------------------------------------------ //
// N/A
// ---------- PUBLIC PROGRAMMING DEFINE ---------------------------------------------------------- //
#define STEP_ANGLE	((2*PI)/200)
#define TIMER_FREQ	1000000U
//#define MOTION_IMPLEMENT_ISR
// ---------- ENUMERATOR DEFINITION -------------------------------------------------------------- //
// N/A
// ---------- TYPEDEF DATA TYPE DEFINITION ------------------------------------------------------- //
// N/A
// ---------- STRUCT OR UNION DATA TYPE DEFINITION ----------------------------------------------- //
// N/A
// ---------- PUBLIC MACRO DEFINITION ------------------------------------------------------------ //
// N/A
// ---------- EXTERN FUNCTION -------------------------------------------------------------------- //
// N/A
// ---------- EXTERN VARIABLE -------------------------------------------------------------------- //
// N/A
// ---------- CLASS DECLARATION ----------------------------------------------------------------- //
namespace Motor {

/*
 *
 */
class StepMotor: public Motor {
public:

	typedef enum {
		STOP,
		ACC,
		RUN,
		DECCEL,
		SPIN
	} MotorState_t;

	typedef enum {
		DIR_CW = 0,
		DIR_CCW
	} MotorDir_t;

	typedef enum {
		MOVE_SPIN,
		MOVE_POSITION
	} MotorMoveMode_t;

	typedef enum {
		CW_LOW = 0,
		CW_HIGH
	} MotorDirLogic_t;

	StepMotor();
	virtual ~StepMotor();

	uint8_t Init(uint8_t dirPin, MotorDirLogic_t dirType);
	uint8_t SetDirPin(uint8_t dirPin);
	uint8_t SetDirLogic(MotorDirLogic_t dirType);
	uint32_t MoveAbs(uint32_t absolute);
	uint32_t MoveInc(uint32_t relative);
	uint32_t Move(int32_t step, uint32_t speed, uint32_t acc, uint32_t dec);
	uint32_t Move(int32_t step);
	uint8_t Stop();
	uint8_t Run(void);
	uint8_t IsMoving() {return this->isRunning;};

	uint32_t GetCurrentPosition() {return this->currentStep;};
	uint8_t GetCurrentDir() {return (uint8_t)this->dir;};
private:
	uint32_t calMinDelay(void);
	uint32_t calC0(void);
	uint32_t calCn(int32_t n);
	uint32_t calSpeed(uint32_t c);
	uint32_t calStepToSpeed(void);
	uint32_t calStepToStop(void);
	uint32_t calAccLimit(uint32_t step);



private:
	uint8_t			dirPin;
	uint8_t			dirState;
	uint8_t			isRunning;
	MotorDir_t		dir;
	MotorState_t	state;
	MotorMoveMode_t	move;
	int32_t 		currentStep;
	int32_t 		targetStep;
	int32_t			n;
	uint32_t		c0;
	uint32_t 		cn;
	uint32_t 		minDelay;
	uint32_t 		stepToSpeed;
	int32_t			stepToDec;
	uint32_t 		stepToStop;
	uint32_t		fref;
};

} /* namespace Motor */

#ifndef MOTION_IMPLEMENT_ISR
extern Motor::StepMotor stepMotor;
#endif

// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //

#endif /* STEPMOTOR_H_ */
