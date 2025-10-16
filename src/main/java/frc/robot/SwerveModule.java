// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  //private static final int EncoderResolution = 4096;
  private static final double WheelCircumprenceMeters = ((3.7 * 0.0254) * Math.PI);
  private static final double gearRatio = 6.75;
  private static final double conversionFactor = WheelCircumprenceMeters / gearRatio;
  //private static final double ModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  //private static final double ModuleMaxAngularAcceleration = Drivetrain.kMaxAngularSpeed / 2;
   // radians per second squared

  private final SparkMax driveMotor;
  private final SparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final CANcoder directionEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController drivePIDController = new PIDController(0, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  // private final ProfiledPIDController turningPIDController =
  //     new ProfiledPIDController(
  //         0.06666,
  //         0,
  //         0,
  //         new TrapezoidProfile.Constraints(
  //             ModuleMaxAngularVelocity, ModuleMaxAngularAcceleration));
  private final PIDController turningPIDController =
  new PIDController(
      0.06666,
      0,
      0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward simpleDriveFeedforward = new SimpleMotorFeedforward(1, 3);
  //private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1.22, 0.22);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID drive motor CANID.
   * @param turningMotorID turning motor CANID.
   * @param encoderID CANcoder CANID.
   */


  public SwerveModule(int driveMotorID, int turningMotorID, int encoderID, boolean IsDriveReversed, boolean IsTurningReversed) {

    driveMotor = new SparkMax(driveMotorID , MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorID , MotorType.kBrushless);

    SparkBaseConfig driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig.idleMode(IdleMode.kBrake);
    driveMotorConfig.inverted(IsDriveReversed);
    driveMotorConfig.encoder.positionConversionFactor(conversionFactor);
    driveMotorConfig.encoder.velocityConversionFactor(conversionFactor / 60);

    SparkBaseConfig turningMotorConfig = new SparkMaxConfig();
    turningMotorConfig.idleMode(IdleMode.kBrake);
    turningMotorConfig.inverted(IsTurningReversed);

    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveMotor.getEncoder();
    directionEncoder = new CANcoder(encoderID);



    turningPIDController.enableContinuousInput(-180, 180);
  }



  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(directionEncoder.getPosition().getValueAsDouble()));
  }

  public Double getAngle(){    
    //final double driveEncoderPos = driveEncoder.getPosition();
    final double directionEncoderPos = directionEncoder.getAbsolutePosition().getValueAsDouble()
    * 360;
    return directionEncoderPos;
  }

  public final double getTurningVoltage(){
    return turningMotor.getOutputCurrent();
  }
  
  public Double getDriveEncoder(){
    return driveEncoder.getPosition();
  }

  public Double getVelocity(){
    return driveEncoder.getVelocity();
  }

  public Double getTurningVelocity(){
    return driveEncoder.getVelocity();
  }

  public void resetDriveEncoder(){
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getVelocity(), new Rotation2d(directionEncoder.getPosition().getValueAsDouble()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = Rotation2d.fromDegrees(getAngle());

  // var encoderRotation = new Rotation2d(directionEncoder.getPosition().getValueAsDouble());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    //desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.

    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = simpleDriveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = turningPIDController.calculate(
      getAngle(), desiredState.angle.getDegrees());
      
     //final double turnFeedforward =
     //  turnFeedforward.calculate(turningPIDController.getSetpoint());

    turningMotor.setVoltage(turnOutput /*+ TurnFeedforward*/);
    driveMotor.setVoltage(driveOutput + driveFeedforward);
    
  }
} 
