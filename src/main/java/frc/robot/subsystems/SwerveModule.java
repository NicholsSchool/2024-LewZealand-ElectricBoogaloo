package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import static frc.robot.Constants.SwerveModuleConstants.*;

public class SwerveModule {

  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivingPIDController;
  private final SparkMaxPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs and configures the driving and turning motors, encoder, PIDs.
   * 
   * @param drivingCANId  The CAN ID of the driving motor.
   * @param turningCANId  The CAN ID of the turning motor.
   * @param angularOffset The angular offset of the module.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double angularOffset) {

    drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, set the SPARKS MAX(s) to a known state before configuration.
    // This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup driving encoder and driving PID controller.
    drivingEncoder = drivingSparkMax.getEncoder();
    drivingPIDController = drivingSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);

    // Setup turning encoder and turning PID controller.
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    turningPIDController = turningSparkMax.getPIDController();
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Calculations required for driving motor conversion factors and feed forward.
    double drivingMotorFreeSpinRPS = DRIVING_MOTOR_FREE_SPIN_RPM / 60;
    double wheelCircumferenceInMeters = WHEEL_DIAMETER_IN_METERS * Math.PI;

    // 45 teeth on the driving wheel's bevel gear.
    // 22 teeth on the first-stage spur gear.
    // 15 teeth on the bevel pinion.
    double drivingMotorReduction = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    double driveWheelFreeSpinRPS = (drivingMotorFreeSpinRPS * wheelCircumferenceInMeters) / drivingMotorReduction;
    double drivingEncoderPositionFactor = (WHEEL_DIAMETER_IN_METERS * Math.PI) / drivingMotorReduction;
    double drivingEncoderVelocityFactor = ((WHEEL_DIAMETER_IN_METERS * Math.PI) / drivingMotorReduction) / 60.0;

    double turningEncoderPositionFactor = (2 * Math.PI);
    double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0;

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but WPILib's swerve APIs want meters and meters per second.
    drivingEncoder.setPositionConversionFactor(drivingEncoderPositionFactor);
    drivingEncoder.setVelocityConversionFactor(drivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder.
    // WPILib's swerve APIs want these vals in radians and radians per second.
    turningEncoder.setPositionConversionFactor(turningEncoderPositionFactor);
    turningEncoder.setVelocityConversionFactor(turningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the REV MAXSwerve Module.
    turningEncoder.setInverted(true);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(0);
    turningPIDController.setPositionPIDWrappingMaxInput(turningEncoderPositionFactor);

    // Set the PID gains for the driving motor.
    drivingPIDController.setP(DRIVING_P);
    drivingPIDController.setI(DRIVING_I);
    drivingPIDController.setD(DRIVING_D);
    drivingPIDController.setFF(DRIVING_FF / driveWheelFreeSpinRPS);
    drivingPIDController.setOutputRange(-1, 1);

    // Set the PID gains for the turning motor.
    turningPIDController.setP(TURNING_P);
    turningPIDController.setI(TURNING_I);
    turningPIDController.setD(TURNING_D);
    turningPIDController.setFF(TURNING_FF);
    turningPIDController.setOutputRange(-1, 1);

    // Set the idle mode (brake or coast).
    drivingSparkMax.setIdleMode(DRIVING_MOTOR_IDLE_MODE);
    turningSparkMax.setIdleMode(TURNING_MOTOR_IDLE_MODE);

    // Current Limiting
    drivingSparkMax.setSmartCurrentLimit(DRIVING_MOTOR_CURRENT_LIMIT);
    turningSparkMax.setSmartCurrentLimit(TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations.
    // Note: If a SPARK MAX browns out it will maintain these configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    chassisAngularOffset = angularOffset;
    desiredModuleState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state (speed and angle) for the module.
   * 
   * @param desiredState The desired state of the swerve modules.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    desiredModuleState = desiredState;
  }

}
