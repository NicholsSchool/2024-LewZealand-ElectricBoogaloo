package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/**
 * Nudges the robot in a specified direction relative to the field.
 */
public class NudgeRobot extends Command {

  SwerveDrive swerveDrive;
  String direction;

  public NudgeRobot(SwerveDrive _swerveDrive, String _direction) {
    swerveDrive = _swerveDrive;
    direction = _direction;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (direction) {
      case "NUDGE FORWARD":
        swerveDrive.drive(0.5, 0.0, 0, true);
        break;
      case "NUDGE BACKWARD":
        swerveDrive.drive(-0.5, 0.0, 0, true);
        break;
      case "NUDGE LEFT":
        swerveDrive.drive(0.0, 0.5, 0, true);
        break;
      case "NUDGE RIGHT":
        swerveDrive.drive(0.0, -0.5, 0, true);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
