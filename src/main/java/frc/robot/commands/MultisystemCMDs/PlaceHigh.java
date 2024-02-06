// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MultisystemCMDs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.Arm.ArmDownCMD;
import frc.robot.commands.Arm.ArmUpCMD;
import frc.robot.commands.Claw.ClawCloseCMD;
import frc.robot.commands.Claw.ClawOpenCMD;
import frc.robot.commands.Wrist.WristPlaceHighCMD;
import frc.robot.commands.Wrist.WristPlaceLowCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHigh extends SequentialCommandGroup {
  Wrist wrist;
  Arm arm;
  Claw claw;
  /** Creates a new PlaceHigh. */
  public PlaceHigh(Wrist wrist, Arm arm, Claw claw) {
    this.wrist = wrist;
    this.arm = arm;
    this.claw = claw;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmUpCMD(arm),
      wrist.wait(ArmConstants.armUpSeconds),
      new WristPlaceHighCMD(wrist)
    );

  }
}