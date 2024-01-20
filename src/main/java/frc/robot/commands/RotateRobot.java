package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

/**
 * Rotate robot chassis to a predefined position relative to the field.
 */
public class RotateRobot extends Command {

  SwerveDrive swerveDrive;
  Double desiredAngle;

  public RotateRobot(SwerveDrive _swerveDrive, Double _desiredAngle) {
    swerveDrive = _swerveDrive;
    desiredAngle = _desiredAngle;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double xSpeed = -RobotContainer.driverOI.getLeftY();
    double ySpeed = -RobotContainer.driverOI.getLeftX();

    double currentYaw = swerveDrive.getYaw();
    System.out.println("yaw:" + currentYaw);
    double difference = desiredAngle - currentYaw;
    double error, angularRotation = 0.0;
    double kP = 1.2;

    if (Math.abs(difference) > 180) {
      error = difference - (360 * (Math.abs(difference) / difference));
    } else {
      error = difference;
    }

    //angularRotation = error / 180 * (Math.PI * kP);
    angularRotation = kP * Math.sin( 0.5 * Math.toRadians( error ) );

    swerveDrive.drive(xSpeed, ySpeed, -angularRotation, true);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
