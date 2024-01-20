package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.HandConstants.*;

public class Hand extends SubsystemBase {

  private CANSparkMax handMotor;
  private DigitalInput handLimitSwitch;

  public Hand() {

    // hand Motor
    handMotor = new CANSparkMax(CANID.HAND_SPARKMAX, MotorType.kBrushless);
    handMotor.restoreFactoryDefaults();
    handMotor.setIdleMode(IdleMode.kBrake);
    handMotor.setInverted(true);

    // limit switch
    handLimitSwitch = new DigitalInput(HAND_LIMIT_SWITCH_DIO_CHANNEL);
  }

  @Override
  public void periodic() {
    RobotContainer.handLS.setBoolean( isPressed() );
  }

  // hand Motors
  public void intake() {
    handMotor.set(INTAKE_SPEED);
  }

  public void outtake() {
    handMotor.set(OUTTAKE_SPEED);
  }

  public void shoot() {
    handMotor.set(SHOOT_SPEED);
  }

  public void stop() {
    handMotor.stopMotor();
  }

  //limit switch
  public boolean isPressed() {
    return !handLimitSwitch.get();
  }
}