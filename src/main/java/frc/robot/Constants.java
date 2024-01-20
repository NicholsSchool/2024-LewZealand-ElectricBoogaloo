package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import java.lang.Math;

public final class Constants {

  // CAN IDs (Controller Area Network)
  public static final class CANID {
    public static final int REAR_RIGHT_DRIVING_SPARKMAX = 28;
    public static final int REAR_RIGHT_TURNING_SPARKMAX = 27;
    public static final int FRONT_RIGHT_DRIVING_SPARKMAX = 26;
    public static final int FRONT_RIGHT_TURNING_SPARKMAX = 25;
    public static final int FRONT_LEFT_DRIVING_SPARKMAX = 24;
    public static final int FRONT_LEFT_TURNING_SPARKMAX = 23;
    public static final int REAR_LEFT_DRIVING_SPARKMAX = 22;
    public static final int REAR_LEFT_TURNING_SPARKMAX = 21;
  }
  // Swerve Drive (Drive Train)
  public static final class SwerveDriveConstants {
    public static final double DRIVETRAIN_WIDTH = 0.653;
    public static final double DRIVETRAIN_LENGTH = 0.653;

    public static final double VIRTUAL_LOW_GEAR_RATE = 0.4;
    public static final double VIRTUAL_HIGH_GEAR_RATE = 0.9;

    public static final double MAX_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),
        new Translation2d(DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2),
        new Translation2d(-DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),
        new Translation2d(-DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2));
  }

  // REV MAXSwerve Modules
  public static final class SwerveModuleConstants {
    public static final int DRIVING_MOTOR_PINION_TEETH = 12; // 12T, 13T, or 14T
    public static final double DRIVING_MOTOR_FREE_SPIN_RPM = 5676; // NEO 550s max RPM
    public static final double WHEEL_DIAMETER_IN_METERS = 0.0762; // 3 inch wheels

    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0.0;
    public static final double DRIVING_D = 0.0;
    public static final double DRIVING_FF = 1.0; // later offset by free spin rate

    public static final double TURNING_P = 1.0;
    public static final double TURNING_I = 0.0;
    public static final double TURNING_D = 0.0;
    public static final double TURNING_FF = 0.0;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 24; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 12; // amps
  }
}
