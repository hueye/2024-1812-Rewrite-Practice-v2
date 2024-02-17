// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants;

public class Drivetrain extends SubsystemBase {

  private SlewRateLimiter strafe = new SlewRateLimiter(4.5*1.75*2.0);
  private SlewRateLimiter translate = new SlewRateLimiter(4.5*1.75*2.0);

  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.FL_DRIVE_ID, DriveConstants.FL_TURN_ID, DriveConstants.FL_CHASSIS_OFFSET);

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.FR_DRIVE_ID, DriveConstants.FR_TURN_ID, DriveConstants.FR_CHASSIS_OFFSET);

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.BL_DRIVE_ID, DriveConstants.BL_TURN_ID, DriveConstants.BL_CHASSIS_OFFSET);

  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.BR_DRIVE_ID, DriveConstants.BR_TURN_ID, DriveConstants.BR_CHASSIS_OFFSET);

  private final Pigeon2 pigeon = new Pigeon2(GlobalConstants.PIGEON_ID);

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS,
  pigeon.getRotation2d(),
  new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  });

  /** Creates a new Drivetrain. */
  public Drivetrain() {
  }

  @Override
  public void periodic() {
    odometry.update(
      pigeon.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      });
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      pigeon.getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()},
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= DriveConstants.MAX_SPEED_MPS;
    ySpeed *= DriveConstants.MAX_SPEED_MPS;
    rot *= DriveConstants.MAX_ANGLE_SPEED;


    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(strafe.calculate(xSpeed), translate.calculate(ySpeed), rot, pigeon.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_MPS);
      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      backLeft.setDesiredState(swerveModuleStates[2]);
      backRight.setDesiredState(swerveModuleStates[3]);

      if(fieldRelative) {
        SmartDashboard.putString("Orientation", "Field Oriented");
      }
      else
      {
        SmartDashboard.putString("Orientation", "Robot Oriented");
      }
  }

  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
    DriveConstants.MAX_SPEED_MPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void resetPigeon() {
    pigeon.reset();
  }

  public double getHeading() {
    return pigeon.getRotation2d().getDegrees();
  }

// figure out why this method is not working
  public StatusSignal<Double> getRoll() {
    return pigeon.getRoll(); 
  }

  public void levelSet(double speed) {
    frontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    backLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    backRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
  }

  public double getTurnRate() {
    return pigeon.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public void turnRobot(double speed) {
    frontLeft.setDesiredState(new SwerveModuleState(-speed, Rotation2d.fromDegrees(-45)));
    frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(45)));
    backLeft.setDesiredState(new SwerveModuleState(-speed, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(45)));
  }

  public void translateRobot(double speed) {
    frontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));
    frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));
    backLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));
    backRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));
  }
}
