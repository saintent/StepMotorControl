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
	void 		Setspeed(uint32_t speed);
	/** \brief Get motor speed
	 *  \return Motor speed in rpm.
	 */
	uint32_t 	GetSpeed(void);

	void 		SetAccelerator(uint32_t acc);
	uint32_t	GetAccelerator(void);

	void		SetDeccelerator(uint32_t dec);
	uint32_t	GetDeccelerator(void);

protected:
	uint32_t speed;		//!< Current motor speed in radian.
	uint32_t maxSpeed;		//!< Max motor speed in radian.
	uint32_t acc;			//!< Motor acceleration in radian.
	uint32_t dec;			//!< Motor deceleration in radian.
};

} /* namespace Motor */
// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //

#endif /* MOTOR_H_ */
