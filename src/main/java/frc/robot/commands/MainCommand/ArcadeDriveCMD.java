package frc.robot.commands.MainCommand;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCMD extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_speedFunction, m_turnFunction;

    public ArcadeDriveCMD(DriveSubsystem driveSubsystem, DoubleSupplier speedFunction, DoubleSupplier turnFunction) {
        m_speedFunction = speedFunction;
        m_turnFunction  = turnFunction;
        m_driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
    }

    @Override
    public void execute() {
        m_driveSubsystem.ArcadeDrive(m_speedFunction.getAsDouble(), m_turnFunction.getAsDouble());
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}