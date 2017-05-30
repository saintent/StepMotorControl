/** \file Motor.h
 *	\brief 
 */

#ifndef MOTOR_H_
#define MOTOR_H_

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
#include "stdint.h"
// ---------- EXTERNAL MODULE INCLUDE ------------------------------------------------------------ //
// N/A
// ---------- PUBLIC PROGRAMMING DEFINE ---------------------------------------------------------- //
// N/A
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
 *	/brief Motor base class
 */
class Motor {
public:
	Motor();
	virtual ~Motor();

	/**	\brief Set speed motor
	 *  \param[in] speed Speed in rpm.
	 */
	void 		Setspeed(float speed);
	/** \brief Get motor speed
	 *  \return Motor speed in rpm.
	 */
	float 	GetSpeed(void);

	void 		SetAccelerator(float acc);
	float	GetAccelerator(void);

	void		SetDeccelerator(float dec);
	float	GetDeccelerator(void);

protected:
	float speed;		//!< Current motor speed in radian.
	float maxSpeed;		//!< Max motor speed in radian.
	float acc;			//!< Motor acceleration in radian.
	float dec;			//!< Motor deceleration in radian.
};

} /* namespace Motor */
// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //

#endif /* MOTOR_H_ */
