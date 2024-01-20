package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private DigitalInput armLimitSwitch;

  public Arm() {

    // arm Motor
    armMotor = new CANSparkMax(CANID.ARM_SPARKMAX, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(true);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, false); //TODO: Put limits back
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) (MAX_ARM_LIMIT) );
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (MIN_ARM_LIMIT) );
    armEncoder = armMotor.getEncoder(Type.kHallSensor, ARM_COUNTS_PER_REV);
    armEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
    armMotor.burnFlash();

    // limit switch
    armLimitSwitch = new DigitalInput(ARM_LIMIT_SWITCH_DIO_CHANNEL);
  }

  @Override
  public void periodic() {
    armValuesToNT();
    if( isPressed() )
      setEncoder(0.0);
  }

  /**
   * Put arm values on network tables.
   */
  public void armValuesToNT() {
    RobotContainer.armEncoder.setDouble(getArmRotations());
    RobotContainer.armLS.setBoolean(isPressed());
  }

  public double getArmRotations() {
    return armEncoder.getPosition();
  }

  public void setEncoder( double position ) {
    armEncoder.setPosition(position);
  }

  public void move( double speed ) {
    if( isPressed() && speed > 0 )
      stop();
    else
      setPower(speed);
  }

  public void setPower( double speed ) {
    armMotor.set(-speed * ARM_SPEED_GOVERNOR);
  }

  public void goToAngle(double desiredPos) {
    double armAngle = getArmRotations();
    move( ARM_P * (desiredPos - armAngle) );
  }

  public void stop() {
    armMotor.stopMotor();
  }

  //limit switch
  public boolean isPressed() {
    return !armLimitSwitch.get();
  }
}