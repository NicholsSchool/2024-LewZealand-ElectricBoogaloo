package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.lang.Math;

public final class Constants {

  // CAN IDs (Controller Area Network)
  public static final class CANID {
    public static final int REAR_RIGHT_DRIVING_SPARKMAX = 10;
    public static final int REAR_RIGHT_TURNING_SPARKMAX = 11;
    public static final int FRONT_RIGHT_DRIVING_SPARKMAX = 12;
    public static final int FRONT_RIGHT_TURNING_SPARKMAX = 13;
    public static final int FRONT_LEFT_DRIVING_SPARKMAX = 14;
    public static final int FRONT_LEFT_TURNING_SPARKMAX = 15;
    public static final int REAR_LEFT_DRIVING_SPARKMAX = 16;
    public static final int REAR_LEFT_TURNING_SPARKMAX = 17;
    public static final int HAND_SPARKMAX = 21;
    public static final int ARM_SPARKMAX = 22;
  }

  public static final class HandConstants {
    public static final double INTAKE_SPEED = -0.6;
    public static final double OUTTAKE_SPEED = 0.3;
    public static final double SHOOT_SPEED = 0.75;
    public static final int HAND_LIMIT_SWITCH_DIO_CHANNEL = 0;
  }

  public static final class ArmConstants {
    public static final double ARM_GEAR_RATIO = 1.0 / (98); // 1:98 gear ratio
    public static final int ARM_COUNTS_PER_REV = 42;
    public static final double POSITION_CONVERSION_FACTOR = ARM_GEAR_RATIO * 2.0 * Math.PI;
    public static final double MIN_ARM_LIMIT = -0.1;
    public static final double MAX_ARM_LIMIT = 1.5;
    public static final double ARM_LOW_POS = 0.0;
    public static final double ARM_MED_POS = 0.5;
    public static final double ARM_HIGH_POS = 1.0;
    public static final double ARM_P = 0.1;
    public static final double ARM_ALLOWED_ERROR = 0.05;
    public static final double ARM_SPEED_GOVERNOR = 0.1;
    public static final int ARM_LIMIT_SWITCH_DIO_CHANNEL = 9;

  }

  // Swerve Drive (Drive Train)
  public static final class SwerveDriveConstants {
    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(26.09);
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(26.06);

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
