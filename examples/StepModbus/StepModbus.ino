/*
 * PP Embedded
 * Stepper motor library and modbus RTU example
 * Pin use on D5 -> Pulse generator
 * Pin dir set by user
 * DIR output can be set by HIGH or LOW by setting on 
 *    stepMotor.Init(pin, MotorDirLogic_t)
 *      where MotorDirLogic_t is "CW_LOW, CW_HIGH"
 *    or
 *    stepMotor.SetDirLogic(MotorDirLogic_t)
 *    
 * DIR pin can be change on the fly by calling
 *    stepMotor.SetDirPin(pin)
 *    
 * Move function :
 *    stepMotor.MoveAbs(abs) -> Absolute move
 *    stepMotor.MoveInc(inc) -> Relative move
 * 
 * Busy flag :
 *    stepMotor.Ismoving()
 *      return  true if moving in progress
 *              false if not move
 * Get current position :
 *    stepMotor.GetCurrentPosition()
 *      return curruent position
 * Set current position :
 *    stepMotor.SetCurrentPosition(position)
 * Stop : stop immedietely
 *    stepMotor.Stop()
 * Set speed : 
 *    stepMotor.SetSpeed(speed)
 * Set ACC :   
 *    stepMotor.SetAccelerator(acc)
 * Set DEC :
 *    stepMotor.SetDeccelerator(dec)
 */

#include "StepMotor.h"
#include <Modbus.h>
#include <ModbusSerial.h>
using namespace Motor;

const int REG_SET_SETP = 0;
const int REG_CURR_STEP = 1;
const int REG_DIR = 2;
const int REG_CURR_DIR = 3;
const int REG_SPEED = 4;
const int REG_ACC = 5;
const int REG_DEC = 6;
const int REG_BUSY = 7;

const int DIR_PIN = 6

void setup() {
  // put your setup code here, to run once:
  stepMotor.Init(DIR_PIN, StepMotor::CW_LOW);
  stepMotor.Setspeed(20);
  stepMotor.SetAccelerator(50);
  stepMotor.SetDeccelerator(50);

  // Config Modbus Serial (port, speed, byte format) 
  mb.config(&Serial, 38400, SERIAL_8N1);
  // Set the Slave ID (1-247)
  mb.setSlaveId(10);  

  mb.addHreg(REG_SET_SETP);
  mb.addHreg(REG_CURR_STEP);
  mb.addHreg(REG_DIR, StepMotor::CW_LOW);
  mb.addHreg(REG_CURR_DIR);
  mb.addHreg(REG_SPEED, 20);
  mb.addHreg(REG_ACC, 50);
  mb.addHreg(REG_DEC, 50);
  mb.addHreg(REG_BUSY, 0);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  mb.task();
  // Map current step
  mb.Hreg(REG_CURR_STEP, stepMotor.GetCurrentPosition())
  // Map current DIR
  mb.Hreg(REG_CURR_DIR, stepMotor.GetCurrentDir())
  // Map BUSY flag
  mb.Hreg(REG_BUSY, stepMotor.Ismoving())

  // Setting attributte from modbus register
  if (stepMotor.Ismoving() == FALSE) {
    // Map speed
    if (stepMotor.GetSpeed() != mb.Hreg(REG_SPEED)) {
        stepMotor.Setspeed(mb.Hreg(REG_SPEED));
    }
    // Map ACC
    if (stepMotor.GetAccelerator() != mb.Hreg(REG_ACC)) {
        stepMotor.SetAccelerator(mb.Hreg(REG_ACC));
    }
    // Map speed
    if (stepMotor.GetDeccelerator() != mb.Hreg(REG_SPEED)) {
        stepMotor.SetDeccelerator(mb.Hreg(REG_SPEED));
    }
    // Execute move cmd
    if (stepMotor.GetCurrentPosition() != mb.Hreg(REG_SET_SETP)) {
        stepMotor.MoveAbs(mb.Hreg(REG_SET_SETP));
    }
  
  }
  
}
