/** \file StepMotor.h
 *	\brief 
 */

#ifndef STEPMOTOR_H_
#define STEPMOTOR_H_

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
#include "stdint.h"
#include "arm_math.h"
#include "Motor.h"
#include "IPWM.h"
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
		DEC,
		SPIN
	} MotorState_t;

	typedef enum {
		DIR_CW,
		DIR_CCW
	} MotorDir_t;

	typedef enum {
		MOVE_SPIN,
		MOVE_POSITION
	} MotorMoveMode_t;

	StepMotor();
	virtual ~StepMotor();

	uint8_t Init();

	uint32_t Move(int32_t step, uint32_t speed, uint32_t acc, uint32_t dec);

	uint8_t Run(void);

	uint32_t GetCurrentPosition() {return this->currentStep;};

private:
	uint32_t calMinDelay(void);
	uint32_t calC0(void);
	uint32_t calCn(int32_t n);
	uint32_t calSpeed(uint32_t c);
	uint32_t calStepToSpeed(void);
	uint32_t calStepToStop(void);
	uint32_t calAccLimit(uint32_t step);



private:
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
extern MOTOR::MotorControl motionControl;
#endif

// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //

#endif /* STEPMOTOR_H_ */
