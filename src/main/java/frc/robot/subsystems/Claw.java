// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {

  DoubleSolenoid clawPiston;

  /** Creates a new Claw. */
  public Claw() {

    clawPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.clawForwardChannel, ClawConstants.clawReverseChannel);

  }

  public void clawSet(Value value) {

    clawPiston.set(value);
    clawPiston.get();

    
  }

  public void toggleClaw() {
    if (clawPiston.get().equals(Value.kForward)) {
      
      clawPiston.set(Value.kReverse);
    
    } else {
    
      clawPiston.set(Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
