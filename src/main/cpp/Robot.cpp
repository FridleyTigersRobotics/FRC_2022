// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

#define DEBUG_LOW_GOAL_SHOOTING       ( 0 )
#define DEBUG_SHOOTER_PID             ( 0 )
#define DEBUG_DISABLE_AIMING_ROTATION ( 0 )
#define DEBUG_MANUAL_HOOD_CONTROL     ( 0 )
#define DELAY_SHOOTER_PID_ENABLE      ( 1 )


void Robot::RobotInit() {

  limelightNetworkTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  limelightNetworkTable->PutNumber( "camMode", 1 );
  limelightNetworkTable->PutNumber( "ledMode", 1 ); //  1	force off


  m_wheelFrontLeft.SetInverted( true );
  m_wheelRearLeft.SetInverted( true );

  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Does not seem to apply to the rotation...
  m_drive.SetDeadband(0.3);

  // TODO : Update these values
  m_rotatePid.SetTolerance(5.0, 10);
  m_rotatePid.SetIntegratorRange(-0.6, 0.6);

  m_rotateGyroPid.SetTolerance(5.0, 10);
  m_rotateGyroPid.SetIntegratorRange(-0.6, 0.6);

  m_shooterMotor.RestoreFactoryDefaults();
  m_shooterMotor.SetInverted( true );

  m_shooterPid.SetP( m_speed_P );
  m_shooterPid.SetI( m_speed_I );
  m_shooterPid.SetD( m_speed_D );
  m_shooterPid.SetIZone( m_speed_Izone );
  m_shooterPid.SetFF( m_speedFF );
  m_shooterPid.SetOutputRange( -1.0, 1.0 );

  m_shooterPid.SetP( m_spool_P, 1 );
  m_shooterPid.SetI( m_spool_I, 1 );
  m_shooterPid.SetD( m_spool_D, 1 );
  m_shooterPid.SetIZone( m_spool_Izone, 1 );
  m_shooterPid.SetFF( m_spool_FF, 1 );
  m_shooterPid.SetOutputRange( -1.0, 1.0, 1 );

#if DEBUG_SHOOTER_PID
  frc::SmartDashboard::PutNumber( "speed_P",     m_speed_P );
  frc::SmartDashboard::PutNumber( "speed_I",     m_speed_I );
  frc::SmartDashboard::PutNumber( "speed_D",     m_speed_D );
  frc::SmartDashboard::PutNumber( "speed_Izone", m_speed_Izone );
  frc::SmartDashboard::PutNumber( "speedFF",     m_speedFF );

  frc::SmartDashboard::PutNumber( "m_spool_P",     m_spool_P );
  frc::SmartDashboard::PutNumber( "m_spool_I",     m_spool_I );
  frc::SmartDashboard::PutNumber( "m_spool_D",     m_spool_D );
  frc::SmartDashboard::PutNumber( "m_spool_Izone", m_spool_Izone );
  frc::SmartDashboard::PutNumber( "m_spool_FF",    m_spool_FF );

  frc::SmartDashboard::PutNumber( "ShooterSpeed", 2000 );
#endif

  m_hoodAngle.ConfigFactoryDefault();

  /* Configure Sensor Source for Pirmary PID */
  m_hoodAngle.ConfigSelectedFeedbackSensor(
    FeedbackDevice::CTRE_MagEncoder_Relative,
    0, 
    10
  );

  /**
   * Configure Talon SRX Output and Sensor direction accordingly
   * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
   * Phase sensor to have positive increment when driving Talon Forward (Green LED)
   */
  m_hoodAngle.SetSensorPhase(false);
  m_hoodAngle.SetInverted(false);

  /* Set relevant frame periods to be at least as fast as periodic rate */
  m_hoodAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_hoodAngle.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  /* Set the peak and nominal outputs */
  m_hoodAngle.ConfigNominalOutputForward(0, 10);
  m_hoodAngle.ConfigNominalOutputReverse(0, 10);
  m_hoodAngle.ConfigPeakOutputForward(0.8, 10);
  m_hoodAngle.ConfigPeakOutputReverse(-0.8, 10);

  /* Set Motion Magic gains in slot0 - see documentation */
  m_hoodAngle.SelectProfileSlot(0, 0);
  m_hoodAngle.Config_kF(0, 0.5, 10);
  m_hoodAngle.Config_kP(0, 0.5, 10);
  m_hoodAngle.Config_kI(0, 0.0001, 10);
  m_hoodAngle.Config_kD(0, 0.0, 10);

  /* Set acceleration and vcruise velocity - see documentation */
  m_hoodAngle.ConfigMotionCruiseVelocity(25000, 10);
  m_hoodAngle.ConfigMotionAcceleration(25000, 10);

  /* Zero the sensor */
  m_hoodAngle.SetSelectedSensorPosition(0, 0, 10);

  /* Zero the climber encoder counter */

  //m_climbEncoder.SetMaxPeriod((units::second_t)1);
  m_climbEncoder.SetMinRate(1);
  m_climbEncoder.SetDistancePerPulse(0.1);
  m_climbEncoder.SetReverseDirection(true);
  m_climbEncoder.SetSamplesToAverage(2);
  m_climbEncoder.Reset();

  /* Zero the IMU yaw axis */
  m_imu.ZeroYaw();

  m_shootTimeLoop=0;
  m_lastLoopAButton=(bool)false;
  m_holdshoot=(bool)false;

  frc::SmartDashboard::PutBoolean( "ReadyToShoot", false );
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

  m_autoState = 0;
  m_initState = true;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  //show IMU gyro values
  //IMUgyroView();

  // Calibrate Hood Angle.
   m_drive.DriveCartesian( 0.0, 0.0, 0.0 );
  if ( m_hoodAngleCalFinished )
  {
    RunOneBallAuto();
  }
  else
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
  // Controls
  bool const   attemptToAim            = m_driverController.GetAButton() || m_logitechController.GetAButton();
  bool const   attemptToAimLowGoal     = m_driverController.GetXButton();
  bool const   intakeEnabled           = m_driverController.GetRightBumper();
  bool const   intakePurge             = m_driverController.GetLeftBumper();
  double const driveSpeedY             = m_driverController.GetLeftY();
  double const driveSpeedX             = -m_driverController.GetLeftX();
  double const driveRotationNoDeadband = -m_driverController.GetRightX();
  double const driveRotation           = ( fabs( driveRotationNoDeadband ) < 0.3 ) ? 
                                           0.0 : 
                                           driveRotationNoDeadband;
  bool   const shootTheBall            = ( m_driverController.GetRightTriggerAxis() + m_logitechController.GetRightTriggerAxis() ) > 0.0;

#if DEBUG_MANUAL_HOOD_CONTROL
  double const releaseClimber          = 0;
  double const engageClimber           = 0;
#else
  double const releaseClimber          = m_logitechController.GetLeftBumper();
  double const engageClimber           = m_logitechController.GetRightBumper();
#endif

  // Debug
#if DEBUG_SHOOTER_PID
  GetAndSetShooterPidControls();
#endif

  // Show IMU gyro values
  //IMUgyroView();
  
  //determine if A button was just released, and should override to spin longer
  if ( !attemptToAim && m_lastLoopAButton )
  {
    m_shootTimeLoop = 0; 
    m_holdshoot     = true;
  }

  if( m_holdshoot )
  {
    // iterative method of robot called every 20ms
    m_shootTimeLoop++;
    if ( m_shootTimeLoop > 25 )
    {
      m_holdshoot = false;
    }
  }

  m_lastLoopAButton = attemptToAim;

  // Cannot aim until hood angle calibration has finished and hold shoot timer is up;
  bool AimingHighGoal = ( attemptToAim && m_hoodAngleCalFinished ) || m_holdshoot;
  bool AimingLowGoal  = ( attemptToAimLowGoal && m_hoodAngleCalFinished );

  // Limelight Control
  if ( AimingHighGoal )
  {
    limelightNetworkTable->PutNumber( "camMode", 0 );
    limelightNetworkTable->PutNumber( "ledMode", 3 ); //  3	force on
  }
  else
  {
    limelightNetworkTable->PutNumber( "camMode", 1 );
    limelightNetworkTable->PutNumber( "ledMode", 1 ); //  1	force off
  }


  // Drive system Control
  bool robotAngleReadyToShoot = false;
  if ( AimingHighGoal )
  {
    double const targetXValue = limelightNetworkTable->GetNumber( "tx", 0.0 );
  #if DEBUG_DISABLE_AIMING_ROTATION
    double const rotation = 0;
    robotAngleReadyToShoot = true;
  #else
    m_rotatePid.SetSetpoint( kRotatePidSetpoint );
    double const rotation = m_rotatePid.Calculate( targetXValue );
  #endif
    m_drive.DriveCartesian( driveSpeedY, driveSpeedX, rotation );
  }
  else
  {
    m_drive.DriveCartesian( driveSpeedY, driveSpeedX, driveRotation );
  }


  // Shooter Control
  bool shooterSpeedReadyToShoot = false;
  if ( AimingLowGoal )
  {
    shooterSpeedReadyToShoot = UpdateShooterSpeedForLowGoal();
  }
  else if ( AimingHighGoal )
  {
    shooterSpeedReadyToShoot = UpdateShooterSpeed();
  }
  else
  {
    m_shooterMotor.Set( 0.0 );
  }

  frc::SmartDashboard::PutNumber( "Shooter Speed",  m_shooterEncoder.GetVelocity() );
  frc::SmartDashboard::PutNumber( "Shooter Output", m_shooterMotor.GetAppliedOutput() );
  frc::SmartDashboard::PutNumber( "Shooter Temp",   m_shooterMotor.GetMotorTemperature() );

  // Hood Control
  bool hoodAngleReadyToShoot = false;
  if ( m_hoodAngleCalFinished )
  {
  #if DEBUG_MANUAL_HOOD_CONTROL
    hoodAngleReadyToShoot = true;
    if ( m_logitechController.GetRightBumper() )
    {
      m_hoodAngle.Set( TalonSRXControlMode::PercentOutput,  0.2 );
    }
    else if ( m_logitechController.GetLeftBumper() )
    {
      m_hoodAngle.Set( TalonSRXControlMode::PercentOutput,  -0.2 );
    }
    else
    {
      m_hoodAngle.Set( TalonSRXControlMode::PercentOutput,  0.0 );
    }
  #else
    if ( AimingLowGoal )
    {
      UpdateShooterHoodAngleForLowGoal();
    }
    else if ( AimingHighGoal )
    {
      UpdateShooterHoodAngleForHighGoal();
    }
    else
    {
      ChangeShooterAngle( 0.0 );
    }
  #endif
  }
  else
  {
    CalibrateShooterAngle();
  }

  frc::SmartDashboard::PutNumber("Hood Output",   m_hoodAngle.GetMotorOutputPercent() );
  frc::SmartDashboard::PutNumber("Hood Position", m_hoodAngle.GetSelectedSensorPosition() );

  frc::SmartDashboard::PutBoolean( "ReadyToShoot", robotAngleReadyToShoot &&
                                                  shooterSpeedReadyToShoot &&
                                                  hoodAngleReadyToShoot );


  // Intake Control
  if ( intakePurge )
  {
    m_intakeSpin.Set( ControlMode::PercentOutput, 1 );
    MoveIntakeOut();
  }
  else if ( intakeEnabled ) 
  {
    m_intakeSpin.Set( ControlMode::PercentOutput, -1 );
    MoveIntakeOut();
  }
  else
  {
    MoveIntakeIn();

    if ( IntakeMovingInward() )
    {
      m_intakeSpin.Set( ControlMode::PercentOutput, -1 );
    }
    else
    {
      m_intakeSpin.Set( ControlMode::PercentOutput, 0.0 );
    }
  }

  // Indexer Control
  double indexerSpeed = 0;
  if ( IntakeMovingInward() )
  {
    m_indexTimer.Reset();
    m_indexTimer.Start();
    m_runningIndexerAfterIntaking = true;
  }

  // Pull balls into indexer when intake is moving inward.
  if ( ( ( AimingHighGoal || AimingLowGoal ) && shootTheBall ) ||
       ( ( IntakeMovingInward() || ( m_runningIndexerAfterIntaking && 
                                     ( m_indexTimer.Get() < (units::time::second_t)1.0 ) ) 
         ) && !IsBallDetected() )
      )
  {
    indexerSpeed = -0.5;
  }
  else
  {
    m_runningIndexerAfterIntaking = false;
    m_indexTimer.Stop();
    m_indexTimer.Reset();
    indexerSpeed = 0.0;
  }

  m_indexer.Set( indexerSpeed );


  // Climber Control
  if ( releaseClimber )
  {
    // int value, max encoder value for climber all the way up, dont overspool winch
    if( m_climbEncoder.Get() < 240000 ) 
    {
      m_climber.Set( ControlMode::PercentOutput, 1.0 );
    }
    else
    {
      m_climber.Set( ControlMode::PercentOutput, 0.0 );
    }
  }
  else if ( engageClimber )
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
  //   Intake Spin  = A
  //   Indexer      = Right Stick
  //   Shooter Hood = Triggers
  //   Climber Up   = X
  //   Climber Down = Y
  
  // WARNING, no limits are enabled in this mode.

  m_indexer.Set( m_driverController.GetRightY() );

  double intakeSpeed = ( m_driverController.GetRightBumper() ? ( 1.0 ) : ( 0.0 ) ) -
                       ( m_driverController.GetLeftBumper()  ? ( 1.0 ) : ( 0.0 ) );
  m_intakeMove.Set( intakeSpeed );

  double hoodSpeed = m_driverController.GetRightTriggerAxis() -
                     m_driverController.GetLeftTriggerAxis();
  m_hoodAngle.Set( TalonSRXControlMode::PercentOutput, hoodSpeed * 0.2 );

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


// Shooter
bool Robot::UpdateShooterSpeed()
{
  bool shooterSpeedReadyToShoot = false;


  //double const targetYValue = limelightNetworkTable->GetNumber( "ty", 0.0 );
#if !DEBUG_SHOOTER_PID
  if ( limelightNetworkTable->GetNumber( "tv", 0.0 ) > 0.0 )
#endif
  {
    double const targetYValue = limelightNetworkTable->GetNumber( "ty", 0.0 );
    frc::SmartDashboard::PutNumber( "targetYValue",  targetYValue );


#if DEBUG_SHOOTER_PID
  double const shooterSpeed = frc::SmartDashboard::GetNumber( "ShooterSpeed", 0 );
#else
  double const shooterSpeed = DetermineShooterSpeedFromTargetPosition( targetYValue );
#endif

    double shooterSpeedAbsError = shooterSpeed - m_shooterEncoder.GetVelocity();
    if ( shooterSpeedAbsError < 100 )
    {
      shooterSpeedReadyToShoot = true;
    }

  #if DELAY_SHOOTER_PID_ENABLE
    if ( m_shooterEncoder.GetVelocity() < ( shooterSpeed - 500 ) )
    {
      m_shooterMotor.Set( 0.8 );
      //m_shooterPid.SetReference( shooterSpeed, rev::ControlType::kVelocity, 1 );
    }
    else
  #endif
    {
      m_shooterPid.SetReference( shooterSpeed, rev::ControlType::kVelocity, 0 );
    }
  }


  return shooterSpeedReadyToShoot;
}

bool Robot::UpdateShooterSpeedForLowGoal()
{
  bool shooterSpeedReadyToShoot = false;
#if DEBUG_LOW_GOAL_SHOOTING
  double const shooterSpeed = frc::SmartDashboard::GetNumber( "ShooterSpeed", 0 );
#else
  double const shooterSpeed = 1500;
#endif

  double shooterSpeedAbsError = shooterSpeed - m_shooterEncoder.GetVelocity();
  if ( shooterSpeedAbsError < 100 )
  {
    shooterSpeedReadyToShoot = true;
  }

#if DELAY_SHOOTER_PID_ENABLE
  if ( m_shooterEncoder.GetVelocity() < ( shooterSpeed - 500 ) )
  {
    m_shooterMotor.Set( 0.8 );
    //m_shooterPid.SetReference( shooterSpeed, rev::ControlType::kVelocity, 1 );
  }
  else
#endif
  {
    m_shooterPid.SetReference( shooterSpeed, rev::ControlType::kVelocity, 0 );
  }

  return shooterSpeedReadyToShoot;
}


bool Robot::UpdateShooterHoodAngleForHighGoal()
{
  bool hoodAngleReadyToShoot = false;

  double targetYValue         = limelightNetworkTable->GetNumber( "ty", 0.0 );
  double shooterAnglePosition = DetermineShooterAngleFromTargetPosition( targetYValue );

  ChangeShooterAngle( shooterAnglePosition );

  hoodAngleReadyToShoot = fabs( m_hoodAngle.GetClosedLoopError() ) < 1000;

  return hoodAngleReadyToShoot;
}

bool Robot::UpdateShooterHoodAngleForLowGoal()
{
  bool hoodAngleReadyToShoot = false;
  double shooterAnglePosition = -92000;
  ChangeShooterAngle( shooterAnglePosition );

  hoodAngleReadyToShoot = fabs( m_hoodAngle.GetClosedLoopError() ) < 1000;

  return hoodAngleReadyToShoot;
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
  return m_ballDetector.GetValue() > 1200;
}


double Robot::DetermineShooterAngleFromTargetPosition( double targetPosition )
{
  // TODO : determine this equation.
  double const hoodPosition = 0;//targetPosition * ( -150000 ) / ( 20.0 ); 
  return hoodPosition;
}

double Robot::DetermineShooterSpeedFromTargetPosition( double targetPosition )
{
  // TODO : determine this equation.
  double const shooterSpeed = 3690 -74.6 * targetPosition + 1.22 * targetPosition * targetPosition;
  return shooterSpeed;
}


void Robot::StopShooterAngle()
{
  if ( m_hoodAngleCalFinished )
  {
    m_hoodAngle.Set( TalonSRXControlMode::PercentOutput, 0.0 );
  }
}

void Robot::ChangeShooterAngle( double position )
{
  // Change this so that 0 = bottom, 1 = top.
  //double const positionScale = 1.0; // TODO : determine this scale.
  //position = std::clamp( position, 0.0, 1.0 );
  /*if ( m_hoodStop.Get() )
  {
    m_hoodAngle.SetSelectedSensorPosition(0, 0, 10);
  }*/

  position = std::clamp( position, kMinHoodPosition, kMaxHoodPosition  );

  m_hoodAngle.Set( ControlMode::MotionMagic, position /* positionScale*/ );
}



void Robot::CalibrateShooterAngle()
{
  // Check if the bottom hood stop has been reached.
  if ( m_hoodStop.Get() )
  {
    // If it is reached, stop motor, zero position, and end calibration.
    m_hoodAngle.Set( TalonSRXControlMode::PercentOutput, 0.0 );
    // Zero Position.
    m_hoodAngle.SetSelectedSensorPosition( 0, 0, 10 );
    m_hoodAngleCalFinished = true;
  }
  else
  {
    // TODO : What speed should this be.
    m_hoodAngle.Set( TalonSRXControlMode::PercentOutput, 0.2 );
  } 
}

void Robot::IMUgyroView()
{
  frc::SmartDashboard::PutNumber("IMU_Yaw",   m_imu.GetYaw() );
  frc::SmartDashboard::PutNumber("IMU_Pitch",   m_imu.GetPitch() );
  frc::SmartDashboard::PutNumber("IMU_Roll",   m_imu.GetRoll() );

  /* Omnimount Yaw Axis Information                                           */
  /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
  AHRS::BoardYawAxis yaw_axis = m_imu.GetBoardYawAxis(); //TODO: Calibrate Omnimount
  frc::SmartDashboard::PutString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
  frc::SmartDashboard::PutNumber(  "YawAxis",              yaw_axis.board_axis );
  frc::SmartDashboard::PutString(  "IMU_FirmwareVersion",  m_imu.GetFirmwareVersion());

  /* These functions are compatible w/the WPI Gyro Class */
  frc::SmartDashboard::PutNumber(  "IMU_TotalYaw",         m_imu.GetAngle());
  frc::SmartDashboard::PutNumber(  "IMU_YawRateDPS",       m_imu.GetRate());
  frc::SmartDashboard::PutNumber(  "Displacement_X",       m_imu.GetDisplacementX() );
  frc::SmartDashboard::PutNumber(  "Displacement_Y",       m_imu.GetDisplacementY() );

}




// Autos
void Robot::RunOneBallAuto()
{
  bool stateDone = false;


  if ( m_hoodAngleCalFinished )
  {
    switch( m_autoState )
    {
      case 0:
      {
        if ( m_initState )
        {
          fmt::print("Init 0\n");
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
        }
        stateDone = DriveForTime( 2.0 );

        break;
      }
      case 1:
      {
        if ( m_initState )
        {
          fmt::print("Init 1\n");
          m_initialAngle = m_imu.GetAngle();
        }
        stateDone = RotateDegrees( m_initialAngle + 180.0 );

        break;
      }
      case 2:
      {
        if ( m_initState )
        {
          fmt::print("Init 2\n");
          limelightNetworkTable->PutNumber( "camMode", 0 );
          limelightNetworkTable->PutNumber( "ledMode", 3 ); //  3	force on
          m_rotatePid.Reset();
          m_rotatePid.SetSetpoint( kRotatePidSetpoint );
        }
        stateDone = AimInAuto();
        break;
      }
      case 3:
      {
        if ( m_initState )
        {
          fmt::print("Init 3\n");
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
        }
        stateDone = AimAndShootInAuto();
        break;
      }
      default:
      {
        limelightNetworkTable->PutNumber( "camMode", 1 );
        limelightNetworkTable->PutNumber( "ledMode", 1 ); //  3	force on
        m_drive.DriveCartesian( 0.0, 0.0, 0.0 );

        // Done, do nothing
        break;
      }
    }

    if ( m_initState )
    {
      m_initState = false;
    }

    if ( stateDone )
    {
      fmt::print( "stateDone {}\n", m_autoState );
      m_autoState++;
      m_initState = true;
    }
  }
  else
  {
    CalibrateShooterAngle();
  }
}

bool Robot::DriveForTime( double time )
{
  if ( m_autoTimer.Get() < (units::time::second_t)time )
  {
    m_drive.DriveCartesian( -0.8, 0.0, 0.0 );
    return false;
  }
  else
  {
    m_drive.DriveCartesian( 0.0, 0.0, 0.0 );
    return true;
  }
}


bool Robot::RotateDegrees( double angle )
{
  double rotationSpeed = m_rotateGyroPid.Calculate( m_imu.GetAngle(), angle );

  if ( m_rotateGyroPid.AtSetpoint() )
  {
    m_drive.DriveCartesian( 0.0, 0.0, 0.0 );
    return true;
  }
  else
  {
    m_drive.DriveCartesian( 0.0, 0.0, -rotationSpeed );
    return false;
  }
}

bool Robot::AimInAuto()
{
    double const targetXValue  = limelightNetworkTable->GetNumber( "tx", 0.0 );
    double const rotationSpeed = m_rotatePid.Calculate( targetXValue );
    m_drive.DriveCartesian( 0.0, 0.0, rotationSpeed );

    bool robotAngleReadyToShoot   = m_rotatePid.AtSetpoint();
    bool hoodAngleReadyToShoot    = UpdateShooterHoodAngleForHighGoal();
    bool shooterSpeedReadyToShoot = UpdateShooterSpeed();

    if ( robotAngleReadyToShoot && 
         hoodAngleReadyToShoot  && 
         shooterSpeedReadyToShoot )
    {
      return true;
    }
    else
    {
      return false;
    }
}

bool Robot::AimAndShootInAuto()
{
  double const targetXValue  = limelightNetworkTable->GetNumber( "tx", 0.0 );
  double const rotationSpeed = m_rotatePid.Calculate( targetXValue );
  m_drive.DriveCartesian( 0.0, 0.0, rotationSpeed );
  UpdateShooterHoodAngleForHighGoal();
  UpdateShooterSpeed();

  if ( m_autoTimer.Get() > (units::time::second_t)2.0 ) 
  {
    m_indexer.Set( -1.0 );
  }
  else
  {
    m_indexer.Set( 0.0 );
  }

  if ( m_autoTimer.Get() < (units::time::second_t)3.0 )
  {
    return false;
  }
  else
  {
    m_shooterMotor.Set( 0.0 );
    m_indexer.Set( 0.0 );
    return true;
  }
}

#if DEBUG_SHOOTER_PID
void Robot::GetAndSetShooterPidControls()
{
  double l_speed_P     = frc::SmartDashboard::GetNumber( "speed_P",     0 );
  double l_speed_I     = frc::SmartDashboard::GetNumber( "speed_I",     0 );
  double l_speed_D     = frc::SmartDashboard::GetNumber( "speed_D",     0 );
  double l_speed_Izone = frc::SmartDashboard::GetNumber( "speed_Izone", 0 );
  double l_speedFF     = frc::SmartDashboard::GetNumber( "speedFF",     0 );

  if ( l_speed_P != m_speed_P )
  {
    m_speed_P = l_speed_P;
    m_shooterPid.SetP( m_speed_P );
  }
  if ( l_speed_I != m_speed_I )
  {
    m_speed_I = l_speed_I;
    m_shooterPid.SetI( m_speed_I );
  }
  if ( l_speed_D != m_speed_D )
  {
    m_speed_D = l_speed_D;
    m_shooterPid.SetD( m_speed_D );
  }
  if ( l_speed_Izone != m_speed_Izone )
  {
    m_speed_Izone = l_speed_Izone;
    m_shooterPid.SetIZone( m_speed_Izone );
  }
  if ( l_speedFF != m_speedFF )
  {
    m_speedFF = l_speedFF;
    m_shooterPid.SetFF( m_speedFF );
  }

  double l_spool_P     = frc::SmartDashboard::GetNumber( "spool_P",     0 );
  double l_spool_I     = frc::SmartDashboard::GetNumber( "spool_I",     0 );
  double l_spool_D     = frc::SmartDashboard::GetNumber( "spool_D",     0 );
  double l_spool_Izone = frc::SmartDashboard::GetNumber( "spool_Izone", 0 );
  double l_spool_FF    = frc::SmartDashboard::GetNumber( "spool_FF",     0 );

  if ( l_spool_P != m_spool_P )
  {
    m_spool_P = l_spool_P;
    m_shooterPid.SetP( m_spool_P, 1 );
  }
  if ( l_spool_I != m_spool_I )
  {
    m_spool_I = l_spool_I;
    m_shooterPid.SetI( m_spool_I, 1 );
  }
  if ( l_spool_D != m_spool_D )
  {
    m_spool_D = l_spool_D;
    m_shooterPid.SetD( m_spool_D, 1 );
  }
  if ( l_spool_Izone != m_spool_Izone )
  {
    m_spool_Izone = l_spool_Izone;
    m_shooterPid.SetIZone( m_spool_Izone, 1 );
  }
  if ( l_spool_FF != m_spool_FF )
  {
    m_spool_FF = l_spool_FF;
    m_shooterPid.SetFF( m_spool_FF, 1 );
  }
}
#endif

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
