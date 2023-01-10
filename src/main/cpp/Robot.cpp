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
  // URL: http://roborio-<TEAM>-frc.local:1181/?action=stream
  //  or: http://10.<TE>.<AM>.2:1181/?action=stream
  // See https://github.com/WPIRoboticsProjects/GRIP.
  m_camera = frc::CameraServer::StartAutomaticCapture("USB Camera 0", 0);

  m_leftMotorA.ConfigFactoryDefault();
  m_rightMotorA.ConfigFactoryDefault();
  m_leftMotorB.ConfigFactoryDefault();
  m_rightMotorB.ConfigFactoryDefault();

  m_Compressor.EnableDigital();
  m_Solenoid.Set(frc::DoubleSolenoid::kOff);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
// This doesn't do anything, but provides a place to add code as neeeded.
void Robot::RobotPeriodic() {}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
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

/**
 * This function is called periodically during operator control.
 */
// This code handles teleop mode, using the game controller left joystick
// values as inputs to the ArcadeDrive() method of DifferentialDrive.
void Robot::TeleopPeriodic()
{
  double x = -m_controller.GetLeftY();
  double y = +m_controller.GetLeftX();
  const bool turbo = m_controller.GetLeftBumper() || m_controller.GetRightBumper();
  const bool flag_up = m_controller.GetXButton();
  const bool flag_down = m_controller.GetYButton();

  auto shape = [](double raw, double mixer = 0.75) -> double
  {
    // Input deadband around 0.0 (+/- range).
    constexpr double range = 0.05;

    constexpr double slope = 1.0 / (1.0 - range);

    if (raw >= -range && raw <= +range)
    {
      raw = 0.0;
    }
    else if (raw < -range)
    {
      raw += range;
      raw *= slope;
    }
    else if (raw > +range)
    {
      raw -= range;
      raw *= slope;
    }

    return mixer * std::pow(raw, 3.0) + (1.0 - mixer) * raw;
  };

  x = shape(x);
  y = shape(y);

  // Limit to 80%, unless turbo button is held.
  if (!turbo)
  {
    x *= 0.8;
    y *= 0.8;
  }

  m_robotDrive.ArcadeDrive(x, y);

  if (flag_up)
  {
    m_Solenoid.Set(frc::DoubleSolenoid::kForward);
  }
  else if (flag_down)
  {
    m_Solenoid.Set(frc::DoubleSolenoid::kReverse);
  }
  else
  {
    m_Solenoid.Set(frc::DoubleSolenoid::kOff);
  }
}

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
  const bool flag_up = m_controller.GetLeftBumper();
  const bool flag_down = m_controller.GetRightBumper();

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

  if (flag_up)
  {
    m_Solenoid.Set(frc::DoubleSolenoid::kForward);
  }
  else if (flag_down)
  {
    m_Solenoid.Set(frc::DoubleSolenoid::kReverse);
  }
  else
  {
    m_Solenoid.Set(frc::DoubleSolenoid::kOff);
  }
}

// This part is boilerplate that came in when the project was generated and can
// just be ignored.
#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
