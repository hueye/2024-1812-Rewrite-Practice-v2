// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.Drivetrain;

public class DriveCMD extends Command {
  Drivetrain drivetrain;
  CommandXboxController controller;
  boolean fieldOriented;
  /** Creates a new DriveCMD. */
  public DriveCMD(Drivetrain drivetrain, CommandXboxController controller, boolean fieldOriented) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.fieldOriented = fieldOriented;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      MathUtil.applyDeadband(controller.getLeftX(), 0.3),
      MathUtil.applyDeadband(controller.getLeftY(), 0.3),
      MathUtil.applyDeadband(controller.getRightX(), 0.3),
      fieldOriented
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
