// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This brings in the "Robot.h" file (look at this other file first).
#include "Robot.h"

// This is used to make it so just "InvertType" may be used as a shorthand, for
// "ctre::phoenix::motorcontrol::InvertType".  Similarly for "NeutralMode".
using ctre::phoenix::motorcontrol::InvertType;
using ctre::phoenix::motorcontrol::NeutralMode;

// The main part of this file consists of the code for each of the methods that
// were declared in "Robot.h".

// No matter the mode(s) the robot is going to be placed into, have all the
// motor controllers start out in a known state (bypassing prior configuration,
// including any configuration set and saved using the "PhoenixTuner" utility).
void Robot::RobotInit()
{
  m_leftMotorA.ConfigFactoryDefault();
  m_rightMotorA.ConfigFactoryDefault();
  m_leftMotorB.ConfigFactoryDefault();
  m_rightMotorB.ConfigFactoryDefault();
}

// This doesn't do anything, but provides a place to add code as neeeded.
void Robot::RobotPeriodic() {}

// This sets the two motor controllers on the right side of the robot so they
// run in the opposite direction.  This is needed because of the physical
// placement of the motors on the robot reverses forward and reverse on one
// side of the drive base.  Also, ensures the motors all coast when set to %0
// power (stopped).
void Robot::TestInit()
{
  m_rightMotorA.SetInverted(InvertType::InvertMotorOutput);
  m_rightMotorB.SetInverted(InvertType::InvertMotorOutput);

  m_leftMotorA.SetNeutralMode(NeutralMode::Coast);
  m_rightMotorA.SetNeutralMode(NeutralMode::Coast);
  m_leftMotorB.SetNeutralMode(NeutralMode::Coast);
  m_rightMotorB.SetNeutralMode(NeutralMode::Coast);
}

// This runs 20 times a second while the robot is in test mode.  First, the
// four "A", "B", "X", and "Y" game controller buttons are read.  Each button
// controls one of the motors.  While pressed, the corresponding motor runs
// "forward" at 20% power; when not pressed, the corresponding motor is stopped
// (which is the same as having the motor run at 0% power).
void Robot::TestPeriodic()
{
  const bool buttonA = m_controller.GetAButton();
  const bool buttonB = m_controller.GetBButton();
  const bool buttonX = m_controller.GetXButton();
  const bool buttonY = m_controller.GetYButton();

  if (buttonA)
  {
    m_leftMotorA.Set(0.2);
  }
  else
  {
    m_leftMotorA.StopMotor();
  }

  if (buttonB)
  {
    m_rightMotorA.Set(0.2);
  }
  else
  {
    m_rightMotorA.StopMotor();
  }

  if (buttonX)
  {
    m_leftMotorB.Set(0.2);
  }
  else
  {
    m_leftMotorB.StopMotor();
  }

  if (buttonY)
  {
    m_rightMotorB.Set(0.2);
  }
  else
  {
    m_rightMotorB.StopMotor();
  }
}

// Much as in TestInit(), both AutonomousInit() and TeleopInit() handle setting
// the right side motor controllers to flip forward and reverse.  Also, this is
// where the two motor controllers on each side are made to act as one, using a
// mechanism known as "Follow".
void Robot::AutonomousInit()
{
  m_leftMotorB.Follow(m_leftMotorA);
  m_rightMotorB.Follow(m_rightMotorA);

  m_leftMotorA.SetInverted(InvertType::None);
  m_rightMotorA.SetInverted(InvertType::InvertMotorOutput);
  m_leftMotorB.SetInverted(InvertType::FollowMaster);
  m_rightMotorB.SetInverted(InvertType::FollowMaster);
}

// Right now, the robot does not do anything at all in autonomous mode.
void Robot::AutonomousPeriodic() {}

// See the comments in AutonomousInit() -- this is the same code.
void Robot::TeleopInit()
{
  m_leftMotorB.Follow(m_leftMotorA);
  m_rightMotorB.Follow(m_rightMotorA);

  m_leftMotorA.SetInverted(InvertType::None);
  m_rightMotorA.SetInverted(InvertType::InvertMotorOutput);
  m_leftMotorB.SetInverted(InvertType::FollowMaster);
  m_rightMotorB.SetInverted(InvertType::FollowMaster);
}

// This code handles teleop mode, using the game controller left joystick
// values as inputs to the ArcadeDrive() method of DifferentialDrive.
void Robot::TeleopPeriodic()
{
  m_robotDrive.ArcadeDrive(
      m_controller.GetLeftY(),
      m_controller.GetLeftX());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

// This part is boilerplate that came in when the project was generated and can
// just be ignored.
#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
