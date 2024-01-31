// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.revrobotics.AbsoluteEncoder;
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

  public CANSparkMax drivingSparkMax;
  public CANSparkMax turningSparkMax;

  public RelativeEncoder drivingEncoder;
  public AbsoluteEncoder turningEncoder;

  public SparkPIDController drivingPIDController;
  public SparkPIDController turningPIDController;
  
    public double chassisAngularOffset = 0;
    public SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** Creates a new SwerveModule. */
  public SwerveModule(int drivingID, int turningID, double chassisAngularOffset) {

    drivingSparkMax = new CANSparkMax(drivingID, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningID, MotorType.kBrushless);

    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    drivingSparkMax.getEncoder();
    turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    drivingEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POS_FACTOR);
    turningEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
    drivingEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

    turningEncoder.setInverted(ModuleConstants.TURN_ENCODER_INVERTED);

    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
    turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);

    //set pid values
    drivingPIDController.setP(ModuleConstants.DRIVE_P);
    drivingPIDController.setI(ModuleConstants.DRIVE_I);
    drivingPIDController.setD(ModuleConstants.DRIVE_D);
    drivingPIDController.setFF(ModuleConstants.DRIVE_FF);
    drivingPIDController.setOutputRange(ModuleConstants.DRIVE_MIN_OUTPUT, ModuleConstants.DRIVE_MAX_OUTPUT);

    turningPIDController.setP(ModuleConstants.TURN_P);
    turningPIDController.setI(ModuleConstants.DRIVE_I);
    turningPIDController.setD(ModuleConstants.DRIVE_D);
    turningPIDController.setFF(ModuleConstants.TURN_FF);
    turningPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);

    drivingSparkMax.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
    turningSparkMax.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE);
    drivingSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);

  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(drivingEncoder.getVelocity(),
      new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(drivingEncoder.getPosition(),
      new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
