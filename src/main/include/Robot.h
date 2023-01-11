// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// Needed for the USB camera to be seen via Ethernet.
#include <cameraserver/CameraServer.h>

// Testbot uses VictorSPX motor controllers.  These need to be the "WPI" class.
// Both of these are electrically connected via the CAN bus, and are being used
// in the context of the FRC "WPILib" coding framework.  The next line is
// used to bring in the necessary information to make use of this two type of
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

// The next two lines are for pneumatics.
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

// The general outine of this "class" is defined in the "TimedRobot" include.
class Robot : public frc::TimedRobot
{
  // USB webcam, through roboRIO.
  cs::UsbCamera m_camera;

  // These resources correspond to the four motor controllers used on Testbot
  // (4x VictorSPX).  We choose the names that start with "m_" so that they are
  // descriptive.  These names describe the mechanism for the motors connected to
  // these motor controllers (left and right drivetrain, A and B motors).
  // The number (1-4) at the end of each line gives the "CAN ID" which matches that
  // configured into the corresponding motor controller.
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

  // The compressor and one double-acting solenoid are connected to the CTRE PCM.
  // The solenoid just happens to be wired into slots 2 and 3.
  frc::Compressor m_Compressor{frc::PneumaticsModuleType::CTREPCM};
  frc::DoubleSolenoid m_Solenoid{frc::PneumaticsModuleType::CTREPCM, 2, 3};

  // This resource corresponds to the game controller that is attached to the
  // driver station; this is how operator controls are tied into the robot code
  // (i.e. this program).  The 0 is because it is the first (and only) game
  // controller used with this robot.
  frc::XboxController m_controller{0};

public:
  // These functions (methods) are part of "TimedRobot"; they run automatically
  // depending on the mode of the robot (autonomous, disabled, teleop, or test)
  // -- the "Init" methods run once at the start of each mode, the "Periodic"
  // methods run 20 times a second as long as the robot is in each mode.  The
  // first two methods ("RobotInit" and "RobotPeriodic") are similar but they
  // run in all modes, after the mode-specific methods for any mode.
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override {}
  void SimulationPeriodic() override {}
};
