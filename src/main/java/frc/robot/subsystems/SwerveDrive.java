package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule frontLeftModule = new SwerveModule(
      CANID.FRONT_LEFT_DRIVING_SPARKMAX,
      CANID.FRONT_LEFT_TURNING_SPARKMAX,
      (-Math.PI / 2));

  private final SwerveModule frontRightModule = new SwerveModule(
      CANID.FRONT_RIGHT_DRIVING_SPARKMAX,
      CANID.FRONT_RIGHT_TURNING_SPARKMAX,
      (0));

  private final SwerveModule rearLeftModule = new SwerveModule(
      CANID.REAR_LEFT_DRIVING_SPARKMAX,
      CANID.REAR_LEFT_TURNING_SPARKMAX,
      (Math.PI));

  private final SwerveModule rearRightModule = new SwerveModule(
      CANID.REAR_RIGHT_DRIVING_SPARKMAX,
      CANID.REAR_RIGHT_TURNING_SPARKMAX,
      (Math.PI / 2));

  // Attitude and Heading Reference System (AHRS)
  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  public double maxPitch = 0;
  public double maxRoll = 0;

  // Setup rate limiters for translation and rotation.
  private double translationalRateLimiter = VIRTUAL_LOW_GEAR_RATE;
  private SlewRateLimiter rotationalRateLimiter = new SlewRateLimiter(ROTATIONAL_SLEW_RATE);

  // Create odometry object for tracking robot pose.
  SwerveDriveOdometry robotOdometry = new SwerveDriveOdometry(
      SWERVE_DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(getContinuousAngle()),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      });

  /**
   * Constructor for a new SwerveDrive Subsystem.
   */
  public SwerveDrive() {
    // Note: NavX calibration takes approximately 3 to 4 seconds.
    navX.calibrate();
  }

  @Override
  public void periodic() {
    // Update the odometry with esitmated robot pose.
    robotOdometry.update(
        Rotation2d.fromDegrees(getContinuousAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });
  }

  /**
   * Set the gear ratio to High Gear
   */
  public void setVirtualHighGear() {
    translationalRateLimiter = VIRTUAL_HIGH_GEAR_RATE;
    System.out.println("Setting virtual gear to HIGH.");
  }

  /**
   * Set the gear ratio to Low Gear
   */
  public void setVirtualLowGear() {
    translationalRateLimiter = VIRTUAL_LOW_GEAR_RATE;
    System.out.println("Setting virtual gear to LOW.");
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * 
   * @return The robot's pose.
   */
  public Pose2d getPose() {
    return robotOdometry.getPoseMeters();
  }

  /** Resets the odometry to the specified pose. */
  public void resetOdometry(Pose2d pose) {
    robotOdometry.resetPosition(
        Rotation2d.fromDegrees(getContinuousAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        }, pose);
  }

  /**
   * Drive the robot based on input from either a joystick or auto routine.
   *
   * @param xSpeed          Speed of the robot in the x direction (forward).
   * @param ySpeed          Speed of the robot in the y direction (sideways).
   * @param angularRotation Speed of the robot's angular rotation.
   * @param fieldRelative   Whether x and y speeds are field relative, or not.
   */
  public void drive(double xSpeed, double ySpeed, double angularRotation, boolean fieldRelative) {

    // Limit the rate of change of the robot's speed before updating module states.
    xSpeed *= MAX_METERS_PER_SECOND * translationalRateLimiter;
    ySpeed *= MAX_METERS_PER_SECOND * translationalRateLimiter;
    angularRotation = rotationalRateLimiter.calculate(angularRotation) * MAX_ANGULAR_SPEED;

    SwerveModuleState[] moduleStates;

    if (fieldRelative) {
      // field relative driving
      moduleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angularRotation,
              Rotation2d.fromDegrees(getContinuousAngle())));
    } else {
      // robot orientated driving
      moduleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, angularRotation));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_METERS_PER_SECOND);

    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    rearLeftModule.setDesiredState(moduleStates[2]);
    rearRightModule.setDesiredState(moduleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setWheelsToXFormation() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_METERS_PER_SECOND);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Reset the AHRS to zero.
   */
  public void resetGyro() {
    System.out.println("Resetting the Gyro.");
    if (navX.getAngleAdjustment() > 0) {
      navX.setAngleAdjustment(0);
    }
    navX.zeroYaw();
  }

  /**
   * Set the angle adjustment for the AHRS.
   * 
   * @param adjustment The angle adjustment in degrees.
   */
  public void setGyroAngleAdjustment(double adjustment) {
    navX.setAngleAdjustment(adjustment);
  }

  /**
   * Get the current continuously accruing angle of the AHRS.
   */
  public double getContinuousAngle() {
    return -navX.getAngle();
  }

  /**
   * Returns the heading of the robot.
   * 
   * @return The current yaw value in degrees (-180 to 180).
   */
  public double getYaw() {
    return navX.getYaw();
  }

  /**
   * Returns the pitch of the robot.
   * 
   * @return The current pitch value in degrees (-180 to 180).
   */
  public double getPitch() {
    return navX.getPitch();
  }

  /**
   * Returns the roll of the robot.
   * 
   * @return The current roll value in degrees (-180 to 180).
   */
  public double getRoll() {
    return navX.getRoll();
  }

  /**
   * Adds the current and max Pitch/Roll to the dashboard.
   */
  public void testPitchRoll() {
    double pitch = getPitch();
    double roll = getRoll();

    if (pitch > maxPitch) {
      maxPitch = pitch;
    }
    if (roll > maxRoll) {
      maxRoll = roll;
    }

    SmartDashboard.putNumber("Pitch: ", pitch);
    SmartDashboard.putNumber("Roll: ", roll);

    SmartDashboard.putNumber("Max Pitch: ", maxPitch);
    SmartDashboard.putNumber("Max Roll: ", maxRoll);
  }

  /**
   * Resets the max Pitch/Roll values.
   */
  public void resetMaxPitchRoll() {
    System.out.println("Resetting max Pitch/Roll.");
    maxPitch = 0;
    maxRoll = 0;
  }

}
