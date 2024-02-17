// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  public static NetworkTable table;

  public double tx;
  public static double ty;
  public boolean tv;
  public double ta;

  public static double currentPipeline;


  public double limePipeline = 1;
  public double April2DPipeline = 0;
  public double April3DPipeline = 9;
  public double cubePipeline = 2;
  public double conePipeline = 3;
  public double humanPlayerPipeline = 5;
  public double rightShelf = 8;
  public static double[] targetRelativePosition;
  public double[] robotRelativePosition;


  /** Creates a new Limelight. */
  public Limelight() {

    table = NetworkTableInstance.getDefault().getTable("limelight");
    currentPipeline = April2DPipeline;
  }

  public Command lightOn() {
    return runOnce( () -> {table.getEntry("led status").setDouble(3); });
  }

  public Command lightOff() {
    return runOnce( () -> {table.getEntry("led status").setDouble(1); });
}

  public Command setLimePipeline() {

    return runOnce( () -> {table.getEntry("led status").setDouble(limePipeline);
    currentPipeline = limePipeline;
    });
  }

  public Command setApril2dPipeline() {

    return runOnce( () -> {table.getEntry("led status").setDouble(April2DPipeline);
    currentPipeline = April2DPipeline;
    });
  }

  public Command setApril3dPipeline() {
    
    return runOnce( () -> {table.getEntry("led status").setDouble(April3DPipeline);
    currentPipeline = April3DPipeline;
    });
  }

  public Command setCubePipeline() {

    return runOnce( () -> {table.getEntry("led status").setDouble(cubePipeline);
    currentPipeline = cubePipeline;
    });
  }

  public Command setConePipeline() {
    
    return runOnce( () -> {table.getEntry("led status").setDouble(conePipeline);
    currentPipeline = conePipeline;
    });
  }

  public Command setHumanPlayerPipeline() {
    
    return runOnce( () -> {table.getEntry("led status").setDouble(humanPlayerPipeline);
    currentPipeline = humanPlayerPipeline;
    });
  }

  public Command setRightShelfPipeline() {
    
    return runOnce( () -> {table.getEntry("led status").setDouble(rightShelf);
    currentPipeline = rightShelf;
    });
  }


  public Command tapeTracking() {
    return Commands.sequence(lightOn(), setLimePipeline());
  }

  public Command april2dTracking() {
    return Commands.sequence(lightOff(), setApril2dPipeline());
  }

  public Command april3dTracking() {
    return Commands.sequence(lightOff(), setApril3dPipeline());
  }

  public Command cubeTracking() {
    return Commands.sequence(lightOff(), setCubePipeline());
  }

  public Command coneTracking() {
    return Commands.sequence(lightOff(), setConePipeline());
  }

  public double angleX() {
    return tx;
  }

  public double angleY() {
    return ty;
  }

  public static double distanceFromTarget() {

    if (currentPipeline == 0 || currentPipeline == 5 || currentPipeline == 8) {
      return targetRelativePosition[2];
    } else if (currentPipeline == 1 || currentPipeline == 3) {
      double LLHeight = 0;
      double targetHeight = 0;
      double combinedAngle = targetHeight - ty;
      return (targetHeight - LLHeight)*Math.tan(Units.degreesToRadians(combinedAngle));
    }

    else {return 0;}
  }

  public double[] april3dCoords() {
    return table.getEntry("botpose").getDoubleArray(new double[6]);
  }

  public boolean trackingAprilTag() {
    if (table.getEntry("pipeline").getDouble(0.0) == 0) {
      return true;
    } else {
      return false;
    }
  }

  public BooleanSupplier targetAcquired() {
    return () -> tv;
  }

  public BooleanSupplier tape() {
    if (currentPipeline == 1) {
      return () -> true;
    } else {
      return () -> false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    try {
      robotRelativePosition = table.getEntry("botpose").getDoubleArray(new double[6]);

      targetRelativePosition = table.getEntry("campose").getDoubleArray(new double[6]);


    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);

  } catch (Exception e) {}
}

  
}
