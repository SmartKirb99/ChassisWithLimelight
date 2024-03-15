// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;


public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {
    setCorrectTarget(); //TODO: make sure limelight is up to date
  }

  public double getDistance() {
    return (LimelightConstants.kGoalHeightMeters - LimelightConstants.kLimelightLensHeightMeters) / Math.tan(LimelightConstants.kMountAngleRadians + Units.degreesToRadians(LimelightHelpers.getTY("")));
  }

  public double distanceToArmAngle(double distance) {
    return ((getDistance() * 10.3) + 34.9);
  }

  public double getTX() {
    return LimelightHelpers.getTX("limelight-shooter");
  }

  public double getTY() {
    return LimelightHelpers.getTY("limelight-shooter");
  }

  public boolean hasCorrectTarget() {
    if (DriverStation.getAlliance().get() == Alliance.Blue && LimelightHelpers.getFiducialID("limelight-shooter") == 7) {
      return true;
    } else if (DriverStation.getAlliance().get() == Alliance.Red && LimelightHelpers.getFiducialID("limelight-shooter") == 4) {
      return true;
    } else {
      return false;
    }
  }

  private void setCorrectTarget() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightHelpers.setPriorityTagID("limelight-shooter", 7);
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID("limelight-shooter", 4);
    } else {
      DriverStation.reportError("Did not get alliance to setup Limelight.", true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
