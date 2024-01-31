// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  public SwerveModule frontLeft = new SwerveModule(
    DriveConstants.FL_DRIVE_ID,
    DriveConstants.FL_TURN_ID,
    DriveConstants.FL_CHASSIS_OFFSET);

  public SwerveModule frontRight = new SwerveModule(
    DriveConstants.FR_DRIVE_ID,
    DriveConstants.FR_TURN_ID,
    DriveConstants.FR_CHASSIS_OFFSET);

  public SwerveModule backLeft = new SwerveModule(
    DriveConstants.BL_DRIVE_ID,
    DriveConstants.BL_TURN_ID,
    DriveConstants.BL_CHASSIS_OFFSET);

  public SwerveModule backRight = new SwerveModule(
    DriveConstants.BR_DRIVE_ID,
    DriveConstants.BR_TURN_ID,
    DriveConstants.BR_CHASSIS_OFFSET);

  public WPI_PigeonIMU pigeon = new WPI_PigeonIMU(GlobalConstants.PIGEON_ID);

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, pigeon.getRotation2d(),
  new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  });

  public SwerveDrive() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      pigeon.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      });


      
  }

}