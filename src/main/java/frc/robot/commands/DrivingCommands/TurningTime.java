//Code below written using Copilot:

package frc.robot.commands.DrivingCommands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TurningTime extends Command {
  // Declare the subsystem and timer dependencies
  private final DriveSubsystem m_drive;
  private final Timer m_timer;

  // Declare the turn speed and duration constants
  private static final double kTurnSpeed = 0.5; // Adjust this value as needed
  private static final double kTurnDuration = 2.665; // Adjust this value as needed

  //  Creates a new TurningTime.
  public TurningTime(DriveSubsystem drive, int i) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_timer = new Timer();
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Turning Command Started");
    // Reset and start the timer
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turn the robot at the specified speed
    m_drive.ArcadeDrive(0, kTurnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turning Command Ended");
    // Stop the robot and the timer
    m_drive.ArcadeDrive(0, 0);
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End the command when the timer reaches the specified duration
    return m_timer.get() >= kTurnDuration;
  }
}
