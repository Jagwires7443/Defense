// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// Testbot has two different types of motor controller: TalonSRX and VictorSPX.
// Both of these are electrically connected via the CAN bus, and are being used
// in the context of the FRC "WPILib" coding framework.  The next two lines are
// used to bring in the necessary information to make use of these two types of
// motor controller.  The company that develops these is "CTRE" (Cross The Road
// Electronics), and the coding framework they offer is known as "Phoenix".
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

// The next three lines are used to bring in the necessary information to use
// different features from the FRC "WPILib" coding framework.  The first is for
// the kind of drive base on Testbot, known as differential drive.  The next is
// for the overall organization of the robot code ("TimedRobot").  The third is
// for the type of game controller used with Testbot.
#include <frc/drive/DifferentialDrive.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

// The general outine of this "class" is defined in the "TimedRobot" include.
class Robot : public frc::TimedRobot
{
  // These resources correspond to the four motor controllers used on Testbot
  // (2x TalonSRX and 2x VictorSPX).  We choose the names that start with "m_".
  // These names describe the mechanism for the motors connected to these motor
  // controllers (left and right drivetrain, A and B motors).  The number (1-4)
  // at the end of each line gives the "CAN ID" which matches that configured
  // into the corresponding motor controller.
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_leftMotorA{1};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_leftMotorB{2};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rightMotorA{3};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rightMotorB{4};

  // This resource does not correspond to any specific piece(s) of hardware but
  // instead, is composed of other already defined resources (above).  Thus, it
  // represents a higher level construct, in this case the drive base.  Since
  // this drive base only directly uses two motor controllers, some other means
  // is going to be used to bring in the other two motor controllers.
  frc::DifferentialDrive m_robotDrive{m_leftMotorA, m_rightMotorA};

  // This resource corresponds to the game controller that is attached to the
  // driver station; this is how operator controls are tied into the robot code
  // (i.e. this program).
  frc::XboxController m_controller{0};

public:
  // These functions (methods) are part of "TimedRobot"; they run automatically
  // depending on the mode of the robot (autonomous, teleop, disabled, or test)
  // -- the "Init" methods run once at the start of each mode, the "Periodic"
  // methods run 20 times a second as long as the robot is in each mode.  The
  // first two methods ("RobotInit" and "RobotPeriodic") are similar but they
  // run in all modes, before the mode-specific methods for any mode.
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

private:
};
