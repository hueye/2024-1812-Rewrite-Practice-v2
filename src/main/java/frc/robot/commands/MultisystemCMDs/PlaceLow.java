// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MultisystemCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Arm.ArmDownCMD;
import frc.robot.commands.Wrist.WristPlaceLowCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceLow extends SequentialCommandGroup {
  Wrist wrist;
  Arm arm;
  /** Creates a new WristLowV3. */
  public PlaceLow(Wrist wrist, Arm arm) {
    this.wrist = wrist;
    this.arm = arm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmDownCMD(arm),
      Commands.waitSeconds(ArmConstants.armDownSeconds),
      new WristPlaceLowCMD(wrist)
    );
  }
}
