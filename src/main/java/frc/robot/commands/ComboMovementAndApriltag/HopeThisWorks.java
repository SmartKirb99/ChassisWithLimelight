// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComboMovementAndApriltag;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.ExamplesAndSuch.*;
import frc.robot.commands.MainCommand.*;
import frc.robot.commands.TuringViewing.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HopeThisWorks extends SequentialCommandGroup {

  
  /** Creates a new HopeThisWorks. */
  public HopeThisWorks(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    

    if (LimelightHelpers.getFiducialID("limelight-shooter") == 7){
      
      addCommands(
        new DriveForwardTimed(drive, 1)
      );
    } else if (LimelightHelpers.getFiducialID("limelight-shooter") == 8) {
      addCommands(
        new TurningTime(drive, 1)
      );
    } else {
      addCommands();
    }
    
  }
}
// 2nd button