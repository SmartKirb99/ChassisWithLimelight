package frc.robot;

import frc.robot.commands.ComboMovementAndApriltag.HopeThisWorks;
import frc.robot.commands.DrivingCommands.DriveForwardTimed;
import frc.robot.commands.DrivingCommands.ForwardAndTurn;
import frc.robot.commands.DrivingCommands.ShortDrive;
import frc.robot.commands.DrivingCommands.TurningTime;
import frc.robot.commands.MainCommand.ArcadeDriveCMD;
import frc.robot.commands.TuringViewing.IdentifyYourselfApriltag;
import frc.robot.commands.TuringViewing.blank;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.*;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final XboxController joystick1 = new XboxController(OIConstants.kDriverJoystickPort);
  private final PS5Controller joystick2 = new PS5Controller(OIConstants.kDriverJoystickPort); //Constant Added :)
  private final Joystick leOperator = new Joystick(OIConstants.kOperatorController);

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  //private NetworkTable m_TV = 
  
  
  
  //Attempting to add buttons...
  
  public RobotContainer() {
    configureButtonBindings();

    m_Chooser.setDefaultOption("Forward and Turning Command", m_MovingForwardandTurningCommand);
    m_Chooser.addOption("Short Driving Command", m_ShortDrivingCommand);
    SmartDashboard.putData(m_Chooser);

    driveSubsystem.setDefaultCommand(new ArcadeDriveCMD(driveSubsystem, () -> joystick1.getLeftY(), () -> joystick1.getLeftX()));  //Replace joystick_ with joystick1 for Xbox Controllers, as for PS5, use joystick2 instead.

    //driveSubsystem.setDefaultCommand(new ArcadeDriveCMD(driveSubsystem, () -> joystick2.getLeftY(), () -> joystick2.getLeftX(), () -> Limelight.getRotationalRate()));
  }

  
  private void configureButtonBindings() {
    //new JoystickButton(joystick1, XboxController.Button.kA.value).onTrue(new DriveForwardTimed(driveSubsystem, 2));
    new JoystickButton(joystick2, PS5Controller.Button.kCross.value).onTrue(new DriveForwardTimed(driveSubsystem, 1)); //X Button
    new JoystickButton(joystick2, PS5Controller.Button.kCircle.value).onTrue(new TurningTime(driveSubsystem, 5)); // O Button
    new JoystickButton(joystick2, PS5Controller.Button.kSquare.value).onTrue(new blank(1));
    new JoystickButton(joystick1, XboxController.Button.kA.value).onTrue(new DriveForwardTimed(driveSubsystem, 1)); //Compatability thingy
    new JoystickButton(joystick1, XboxController.Button.kB.value).onTrue(new TurningTime(driveSubsystem, 5));
    new JoystickButton(joystick1, XboxController.Button.kX.value).onTrue(new blank(1));

    // Operating Buttons on the Button Board

    new JoystickButton(leOperator, 1).onTrue(new IdentifyYourselfApriltag());
    new JoystickButton(leOperator, 2).onTrue(new HopeThisWorks(driveSubsystem));
    new JoystickButton(leOperator, 5).onTrue(new ForwardAndTurn(driveSubsystem));
    
  } 

  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

  private final Command m_MovingForwardandTurningCommand = new ForwardAndTurn(driveSubsystem);
  private final Command m_ShortDrivingCommand = new ShortDrive(driveSubsystem);
  
  private final SendableChooser<Command> m_Chooser = new SendableChooser<>();
  
  
}
  // A simple auto routine that drives forward a specified distance, and then stops.
  
  
  /*private final Command m_simpleAuto =
      new DriveDistance(
          AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed, m_robotDrive);

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command m_complexAuto = new ComplexAuto(m_robotDrive, m_hatchSubsystem);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>(); **/