package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class DriveConstants {
    public static final int kLeftMasterPort = 0;
    public static final int kRightMasterPort = 3;
    public static final int kLeftFollowerPort = 1;
    public static final int kRightFollowerPort = 2;
    public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;

    public static final double kAutoDriveForwardSpeed = 0.25;
    public static final double kAutoDriveForwardDistance = 1.5;
  }

  public static final class OIConstants {
    public static final int kDriverJoystickPort = 0;

    public static final int kArcadeDriveSpeedAxis = 1;
    public static final int kArcadeDriveTurnAxis = 3;

    public static int kOperatorController = 1;
  }

  public static final class LimelightConstants {

	public static double kGoalHeightMeters = Units.inchesToMeters(57.88);
    public static double kLimelightLensHeightMeters = 0.2;
    public static double kMountAngleRadians = Units.degreesToRadians(0);

  }


}