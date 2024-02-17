// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  
  DoubleSolenoid armPiston;

  /** Creates a new Arm. */
  public Arm() {

  armPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, ArmConstants.armForwardChannel, ArmConstants.armReverseChannel);
  
  }

  public void armSet(Value value) {
    armPiston.set(value);
  }

  public Value armGet() {
    return armPiston.get();
  }

  public void toggleArm() {
    if (armPiston.get().equals(Value.kForward)) {
      
      armPiston.set(Value.kReverse);
    
    } else {

      armPiston.set(Value.kForward);

      }
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}