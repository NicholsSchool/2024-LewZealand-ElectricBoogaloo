// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class TurnMet extends Command {
  SwerveDrive swerveDrive;
  double desiredAngle;
  double angle;

  public TurnMet(SwerveDrive swerveDrive, double desiredAngle, double angle) {
    this.swerveDrive = swerveDrive;
    this.angle = angle;
    this.desiredAngle = desiredAngle;

    addRequirements(this.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = swerveDrive.getYaw();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - desiredAngle) < 20.0;
  }
}
