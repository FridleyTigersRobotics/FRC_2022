// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>


void Robot::RobotInit() {

  limelightNetworkTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  //bool   targetValid                  = limelightNetworkTable->GetNumber("tv",false);
  //double targetOffsetAngle_Horizontal = limelightNetworkTable->GetNumber("tx",0.0);
  //double targetOffsetAngle_Vertical   = limelightNetworkTable->GetNumber("ty",0.0);
  //double targetArea                   = limelightNetworkTable->GetNumber("ta",0.0);
  //double targetSkew                   = limelightNetworkTable->GetNumber("ts",0.0);

  m_wheelFrontLeft.SetInverted( true );
  m_wheelRearLeft.SetInverted( true );

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Does not seem to apply to the rotation...
  m_drive.SetDeadband(0.3);

  // TODO : Update these values
  m_rotatePid.SetTolerance(5.0, 10);
  m_rotatePid.SetIntegratorRange(-0.6, 0.6);

  m_shooter.RestoreFactoryDefaults();
  m_pidController.SetP( 6e-5 );
  m_pidController.SetI( 0.0 );
  m_pidController.SetD( 0.0 );
  m_pidController.SetIZone( 0.0 );
  m_pidController.SetFF( 0.00025 );
  m_pidController.SetOutputRange( -1.0, 1.0 );

  m_shooterAngle.ConfigFactoryDefault();

  /* Configure Sensor Source for Pirmary PID */
  m_shooterAngle.ConfigSelectedFeedbackSensor(
    FeedbackDevice::CTRE_MagEncoder_Relative,
    0, 
    10
  );

  /**
   * Configure Talon SRX Output and Sensor direction accordingly
   * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
   * Phase sensor to have positive increment when driving Talon Forward (Green LED)
   */
  m_shooterAngle.SetSensorPhase(false);
  m_shooterAngle.SetInverted(false);

  /* Set relevant frame periods to be at least as fast as periodic rate */
  m_shooterAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_shooterAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  /* Set the peak and nominal outputs */
  m_shooterAngle.ConfigNominalOutputForward(0, 10);
  m_shooterAngle.ConfigNominalOutputReverse(0, 10);
  m_shooterAngle.ConfigPeakOutputForward(0.8, 10);
  m_shooterAngle.ConfigPeakOutputReverse(-0.8, 10);

  /* Set Motion Magic gains in slot0 - see documentation */
  m_shooterAngle.SelectProfileSlot(0, 0);
  m_shooterAngle.Config_kF(0, 0.5, 10);
  m_shooterAngle.Config_kP(0, 0.5, 10);
  m_shooterAngle.Config_kI(0, 0.0001, 10);
  m_shooterAngle.Config_kD(0, 0.0, 10);

  /* Set acceleration and vcruise velocity - see documentation */
  m_shooterAngle.ConfigMotionCruiseVelocity(25000, 10);
  m_shooterAngle.ConfigMotionAcceleration(25000, 10);

  /* Zero the sensor */
  m_shooterAngle.SetSelectedSensorPosition(0, 0, 10);

  /* Zero the climber encoder counter */
  m_climbEncoder.Reset();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {

  // Calibrate Hood Angle.
  if ( !m_hoodAngleCalFinished )
  {
    CalibrateShooterAngle();
  }

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {

}

// TODO : Check signs of all motors.

void Robot::TeleopPeriodic() {
  bool targetValid = limelightNetworkTable->GetNumber("tv",false);

  // Cannot aim until hood angle calibration has finished.
  bool Aiming        = m_driverController.GetAButton() && m_hoodAngleCalFinished;
  bool intakeEnabled = m_driverController.GetRightBumper();

  // Limelight Control
  // nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",<value>);
  // limelightNetworkTable->PutNumber("ledMode",<value>);
  //  0	use the LED Mode set in the current pipeline
  //  1	force off
  //  2	force blink
  //  3	force on
  if ( Aiming )
  {
    limelightNetworkTable->PutNumber( "ledMode", 0 );
  }
  else
  {
    limelightNetworkTable->PutNumber( "ledMode", 1 );
  }


  // Drive system Control
  if ( Aiming )
  {
    double const ySpeed       = m_driverController.GetLeftY();
    double const xSpeed       = -m_driverController.GetLeftX();
    double const targetXValue = limelightNetworkTable->GetNumber( "tx", 0.0 );
    double const rotation     = m_rotatePid.Calculate( targetXValue, 0.0 );
    m_drive.DriveCartesian( ySpeed, xSpeed, rotation );
  }
  else
  {
    double const ySpeed   = m_driverController.GetLeftY();
    double const xSpeed   = -m_driverController.GetLeftX();
    double       rotation = -m_driverController.GetRightX();

    // Deadband does not apply to rotation for some reason...
    if ( fabs( rotation ) < 0.3 )
    {
      rotation = 0.0;
    }

    m_drive.DriveCartesian( ySpeed, xSpeed, rotation );
  }


  // Shooter Control
  if ( Aiming )
  {
    // TODO : Determine Velocity to use.
    m_pidController.SetReference( -1500, rev::ControlType::kVelocity );
  }
  else
  {
    m_pidController.SetReference( 0, rev::ControlType::kVelocity );
  }
  frc::SmartDashboard::PutNumber( "Shooter Speed:",  m_encoder.GetVelocity() );
  frc::SmartDashboard::PutNumber( "Shooter Output:", m_shooter.GetAppliedOutput() );


  // Hood Angle
  if ( m_hoodAngleCalFinished )
  {
    double targetYValue         = limelightNetworkTable->GetNumber( "ty", 0.0 );
    double shooterAnglePosition = DetermineShooterAngleFromTargetPosition( targetYValue );
    ChangeShooterAngle( shooterAnglePosition );
  }
  else
  {
    CalibrateShooterAngle();
  }
  frc::SmartDashboard::PutNumber("Hood Output",   m_shooterAngle.GetMotorOutputPercent() );
  frc::SmartDashboard::PutNumber("Hood Velocity", m_shooterAngle.GetSelectedSensorVelocity() );
  frc::SmartDashboard::PutNumber("Hood Position", m_shooterAngle.GetSelectedSensorPosition() );
  frc::SmartDashboard::PutNumber("Hood error   ", m_shooterAngle.GetClosedLoopError(0) );


  // Intake Control
  if ( intakeEnabled ) 
  {
    m_intakeSpin.Set( ControlMode::PercentOutput, -1 );
    MoveIntakeOut();
  }
  else
  {
    m_intakeSpin.Set( ControlMode::PercentOutput, 0.0 );
    MoveIntakeIn();

    // Pull balls into indexer when intake is moving inward.
    if ( IntakeMovingInward() && 
         !IsBallDetected() )
    {
      m_indexer.Set(-0.5 );
    }
    else
    {
      m_indexer.Set( 0.0 );
    }
  }


  // Indexer Control
  if ( Aiming )
  {
    m_indexer.Set( -m_driverController.GetRightTriggerAxis() );
  }


  // Climber Control
  frc::SmartDashboard::PutNumber("Climber Index",   m_climbEncoder.Get() );
  if ( m_driverController.GetYButton() )//Y is release climb
  {
    if( m_climbEncoder.Get() < 50 ) // int value, max encoder value for climber all the way up, dont overspool winch TODO: set this value
    {
      m_climber.Set( ControlMode::PercentOutput, 1.0 );
    } 
    else
    {
      m_climber.Set( ControlMode::PercentOutput, 0.0 );
    }
  }
  else if ( m_driverController.GetXButton() )//X is climb up
  {
    if( m_climbStop.Get() ) // check if bottomed out climber
    {
      m_climber.Set( ControlMode::PercentOutput, 0.0 );
      m_climbEncoder.Reset(); //re-zero encoder count
    }
    else
    {
      m_climber.Set( ControlMode::PercentOutput, -1.0 );
    }
  } 
  else
  {
    m_climber.Set( ControlMode::PercentOutput, 0.0 );
  }
}


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
  // TEST Controls:
  //   Intake Arm   = Bumpers
  //   Indexer      = Right Stick
  //   Shooter Hood = Triggers
  
  // WARNING, no limits are enabled in this mode.

  m_indexer.Set( m_driverController.GetRightY() );

  double intakeSpeed = ( m_driverController.GetRightBumper() ? ( 1.0 ) : ( 0.0 ) ) -
                       ( m_driverController.GetLeftBumper()  ? ( 1.0 ) : ( 0.0 ) );
  m_intakeMove.Set( intakeSpeed );

  double hoodSpeed = m_driverController.GetRightTriggerAxis() -
                     m_driverController.GetLeftTriggerAxis();
  m_shooterAngle.Set( TalonSRXControlMode::PercentOutput, hoodSpeed * 0.2 );

  if ( m_driverController.GetAButton() )
  {
    m_intakeSpin.Set( ControlMode::PercentOutput, -1 );
  }
  else
  {
    m_intakeSpin.Set( ControlMode::PercentOutput, 0 );
  }

  if ( m_driverController.GetXButton() )
  {
    m_climber.Set( ControlMode::PercentOutput, 1.0 );
  }
  else if ( m_driverController.GetYButton() )
  {
    m_climber.Set( ControlMode::PercentOutput, -1.0 );
  } 
  else
  {
    m_climber.Set( ControlMode::PercentOutput, 0.0 );
  }
}

// Intake
void Robot::SetIntakeMoveSpeed( double speed ) {
  bool intake_OutStop = !m_intakeOutStop.Get();
  bool intake_InStop  = !m_intakeInStop.Get();

  if ( ( speed < 0 && intake_InStop ) ||
       ( speed > 0 && intake_OutStop ) )
  {
    m_intakeMove.Set( 0 );
  }
  else
  {
    m_intakeMove.Set( speed );
  }
}

void Robot::MoveIntakeOut() {
  SetIntakeMoveSpeed( 1.0 );
}

void Robot::MoveIntakeIn() {
  SetIntakeMoveSpeed( -1.0 );
}

bool Robot::IntakeMovingInward() { 
  return m_intakeMove.Get() < 0.0;
}


bool Robot::IsBallDetected() {
  // TODO : update this value for th sensor.
  return m_ballDetector.GetValue() > 1200;
}


double Robot::DetermineShooterAngleFromTargetPosition( double targetPosition )
{
  // TODO : determine this equation.
  double const hoodPosition = targetPosition * ( -150000 ) / ( 20.0 ); 
  return hoodPosition;
}


void Robot::StopShooterAngle()
{
  if ( m_hoodAngleCalFinished )
  {
    m_shooterAngle.Set( TalonSRXControlMode::PercentOutput, 0.0 );
  }
}

void Robot::ChangeShooterAngle( double position )
{
  // Change this so that 0 = bottom, 1 = top.
  //double const positionScale = 1.0; // TODO : determine this scale.
  //position = std::clamp( position, 0.0, 1.0 );
  if ( m_hoodStop.Get() )
  {
    m_shooterAngle.SetSelectedSensorPosition(0, 0, 10);
  }

  position = std::clamp( position, kMinHoodPosition, kMaxHoodPosition  );

  m_shooterAngle.Set( ControlMode::MotionMagic, position /* positionScale*/ );
}



void Robot::CalibrateShooterAngle()
{
  // Check if the bottom hood stop has been reached.
  if ( m_hoodStop.Get() )
  {
    // If it is reached, stop motor, zero position, and end calibration.
    m_shooterAngle.Set( TalonSRXControlMode::PercentOutput, 0.0 );
    // Zero Position.
    m_shooterAngle.SetSelectedSensorPosition( 0, 0, 10 );
    m_hoodAngleCalFinished = true;
  }
  else
  {
    // TODO : What speed should this be.
    m_shooterAngle.Set( TalonSRXControlMode::PercentOutput, 0.2 );
  } 
}




#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
