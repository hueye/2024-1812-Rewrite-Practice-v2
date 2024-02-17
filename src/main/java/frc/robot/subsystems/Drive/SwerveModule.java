// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  CANSparkFlex driveMotor;
  CANSparkMax turnMotor;

  RelativeEncoder driveEncoder;
  AbsoluteEncoder turnEncoder;

  SparkPIDController drivePIDController;
  SparkPIDController turnPIDController;

  double chassisAngularOffset = 0;
  SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int turnID, double chassisAngularOffset) {
    driveMotor = new CANSparkFlex(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePIDController = driveMotor.getPIDController();
    turnPIDController = turnMotor.getPIDController();

    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POS_FACTOR);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);
    turnEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
    turnEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

    drivePIDController.setP(ModuleConstants.DRIVE_P);
    drivePIDController.setI(ModuleConstants.DRIVE_I);
    drivePIDController.setD(ModuleConstants.DRIVE_D);
    drivePIDController.setFF(ModuleConstants.DRIVE_FF);
    drivePIDController.setOutputRange(ModuleConstants.DRIVE_MIN_OUTPUT, ModuleConstants.DRIVE_MAX_OUTPUT);

    turnPIDController.setP(ModuleConstants.TURN_P);
    turnPIDController.setI(ModuleConstants.TURN_I);
    turnPIDController.setD(ModuleConstants.TURN_D);
    turnPIDController.setFF(ModuleConstants.TURN_FF);
    turnPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);

    driveMotor.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
    turnMotor.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE);
    driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    turnMotor.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

    driveMotor.burnFlash();
    turnMotor.burnFlash();


    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);
  }


  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(),
    new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(),
    new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState swerveModuleStates) {

    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = swerveModuleStates.speedMetersPerSecond;
    correctedDesiredState.angle = swerveModuleStates.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
  
    //eliminates rotation greater than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
    new Rotation2d(turnEncoder.getPosition()));

    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    desiredState = optimizedDesiredState;
  }

  public void setDesiredStatenoOpt(SwerveModuleState desiredState) {

    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    drivePIDController.setReference(correctedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnPIDController.setReference(correctedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    correctedDesiredState = desiredState;
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }
  
  public double getWheelVelocity()
  {
    return driveEncoder.getVelocity();
  }
  
  public double getDesiredVelocity()
  {
    return desiredState.speedMetersPerSecond;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
