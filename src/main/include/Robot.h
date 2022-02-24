// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

#include <frc/controller/PIDController.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Encoder.h>

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot {
 public:
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

  void SetIntakeMoveSpeed( double );
  void MoveIntakeOut();
  void MoveIntakeIn();
  bool IntakeMovingInward();
  bool IsBallDetected();
  double DetermineShooterAngleFromTargetPosition( double );
  void StopShooterAngle();
  void ChangeShooterAngle( double );
  void CalibrateShooterAngle();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string    kAutoNameDefault = "Default";
  const std::string    kAutoNameCustom = "My Auto";
  std::string          m_autoSelected;
  bool                 m_hoodAngleCalFinished{false};

  std::shared_ptr<nt::NetworkTable> limelightNetworkTable;

  // TODO : Update these PID values
  double               kRotateP{ 0.05 };
  double               kRotateI{ 0.0001 };
  double               kRotateD{ 0.0 };

  double               kMaxHoodPosition{ 0 };
  double               kMinHoodPosition{ -150000 };
  frc2::PIDController  m_rotatePid{ kRotateP, kRotateI, kRotateD };



  frc::XboxController  m_driverController{0};

  // PWM Channels
  // 0 - Intake, in-out
  // 1 - Front Right Wheel
  // 2 - Indexer
  // 3 - Front Left Wheel
  // 4 - Rear Right Wheel
  // 5 - Rear Left Wheel
  frc::PWMVictorSPX m_intakeMove     {0};
  frc::PWMVictorSPX m_wheelFrontRight{1};
  frc::PWMVictorSPX m_indexer        {2}; 
  frc::PWMVictorSPX m_wheelFrontLeft {3};
  frc::PWMVictorSPX m_wheelRearRight {4};
  frc::PWMVictorSPX m_wheelRearLeft  {5};

  frc::MecanumDrive m_drive{ 
    m_wheelFrontLeft,
    m_wheelRearLeft,
    m_wheelFrontRight,
    m_wheelRearRight };

  // CAN IDs
  // 0 - Roborio
  // 1 - Shooter Angle
  // 2 - Shooter Wheel
  // 3 - Climber
  // 4 - Intake Wheels
  // 5 - PDP
  TalonSRX         m_shooterAngle{1};
  rev::CANSparkMax m_shooter     {2, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
  VictorSPX        m_climber     {3}; 
  VictorSPX        m_intakeSpin  {4}; 

  // DIO Channels
  // 0 - Out Stop
  // 1 - In Stop
  // 2 - Shooter Hood Stop
  // 3 - Climber bottom out
  // 4 - climbing encoder channel A quad
  // 5 - climbing encoder channel B quad TODO: verify DIO channels
  frc::DigitalInput m_intakeOutStop{0};
  frc::DigitalInput m_intakeInStop {1};
  frc::DigitalInput m_hoodStop     {2};
  frc::DigitalInput m_climbStop    {3};
  frc::Encoder      m_climbEncoder {4,5, false, frc::CounterBase::EncodingType::k4X };

  // Analog Channels
  // 0 - Ball Detection
  frc::AnalogInput m_ballDetector{0};


  rev::SparkMaxPIDController   m_pidController = m_shooter.GetPIDController();
  rev::SparkMaxRelativeEncoder m_encoder       = m_shooter.GetEncoder();

};
