// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TuringViewing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.*;


public class IdentifyYourselfApriltag extends Command {
  /** Creates a new IdentifyYourselfApriltag command. */
  public IdentifyYourselfApriltag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-shooter")){
      System.out.println("The ID is " + LimelightHelpers.getFiducialID("limelight-shooter"));
    } else {
      System.out.println("There is no visible Apriltag");
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
// Ah um well you see... this is meant to give the ID when the 1st button on the button board is pressed