/*
 * MotorControl.h
 *
 *  Created on: Oct 2, 2016
 *      Author: Prustya
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
#include "stdint.h"
// ---------- EXTERNAL MODULE INCLUDE ------------------------------------------------------------ //
// N/A
// ---------- PUBLIC PROGRAMMING DEFINE ---------------------------------------------------------- //
//#define MOTION_IMPLEMENT_ISR
/*! \Brief Frequency of timer1 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1-FREQ 460750)
#define T1_FREQ 1000000

//! Number of (full)steps per round on stepper motor in use.
#define FSPR 200

#define SPR FSPR

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000
// ---------- ENUMERATOR DEFINITION -------------------------------------------------------------- //
namespace MOTOR {
enum Direction {
	Direction_CW, Direction_CCW
};

enum SpeedProfileState {
	SPS_Stop, SPS_Accel, SPS_Decel, SPS_Run
};
}
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
namespace MOTOR {
/*
 *
 */
class MotorControl {
public:
	MotorControl();
	virtual ~MotorControl();

	void Init();
	void begin();

	void Move(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

	static void Callback(MotorControl* obj);

private:
	void Process(void);
	void StartTimer(void);
	void StopTimer();

private:
	SpeedProfileState runState;
	Direction direction;
	uint32_t stepPosition;
	uint32_t stepDelay;
	uint32_t decelStart;
	int32_t decelVal;
	int32_t minDelay;
	int32_t accelCount;

	uint32_t lastAccelDelay;
	uint32_t stepCount;
	int32_t rest;
};

}

#ifndef MOTION_IMPLEMENT_ISR
extern MOTOR::MotorControl motionControl;
#endif

// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //
#endif /* MOTORCONTROL_H_ */

