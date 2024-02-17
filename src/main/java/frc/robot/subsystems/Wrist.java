// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

  CANSparkMax wristMotor;
  AbsoluteEncoder wristEncoder;
  SparkPIDController controller;
  double desiredAngle;

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);
    
    wristMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setSmartCurrentLimit(40);

    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristEncoder.setInverted(true);
    wristEncoder.setPositionConversionFactor(360);
    wristEncoder.setVelocityConversionFactor(wristEncoder.getPositionConversionFactor() / 60.0);
   
    controller = wristMotor.getPIDController();
    controller.setFeedbackDevice(wristEncoder);
    controller.setP(WristConstants.wristP);
    controller.setI(WristConstants.wristI);
    controller.setD(WristConstants.wristD);
    controller.setFF(WristConstants.wristFF);

    controller.setOutputRange(WristConstants.wristOutputMin, WristConstants.wristOutputMax);
    controller.setReference(0, ControlType.kDutyCycle);
    //initially wrist starts at 0 degrees

    wristMotor.burnFlash();
  }

  public void wristSetAngle(double desiredAngle) {
  
    controller.setReference(desiredAngle, ControlType.kDutyCycle);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("wrist angle", wristEncoder.getPosition());
  }


}
