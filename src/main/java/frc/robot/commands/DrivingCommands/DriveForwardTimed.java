
package frc.robot.commands.DrivingCommands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForwardTimed extends Command {

    private final DriveSubsystem m_drive;
    private final Timer m_timer;

    private static final double kMoveSpeed = -0.4; //Adjust as needed
    private static final double kMoveDuration = 1.0; //Adjust as needed

    public DriveForwardTimed(DriveSubsystem drive, double t) {
        m_drive = drive;
        m_timer = new Timer();
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        System.out.println("DriveForwardCmd started!");
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_drive.ArcadeDrive(kMoveSpeed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.ArcadeDrive(0, 0);
        System.out.println("DriveForwardCmd ended!");
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= kMoveDuration;
    }   
}