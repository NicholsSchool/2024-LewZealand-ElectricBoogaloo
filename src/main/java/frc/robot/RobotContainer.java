package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

  // Subsystems
  public final SwerveDrive swerveDrive;

  // OI (Operator Interface) controllers
  public static CommandXboxController driverOI;
  public static CommandXboxController operatorOI;
  public static XboxController driverRumbler;
  public static XboxController operatorRumbler;

  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();

    // OI (Operator Interface) Controllers & Rumblers
    driverOI = new CommandXboxController(1);
    driverRumbler = new XboxController(1);
    operatorOI = new CommandXboxController(0);
    operatorRumbler = new XboxController(0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // DRIVER Left & Right Stick: Translational and rotational robot movement.
    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY() * 0.8, 0.05),
                -MathUtil.applyDeadband(driverOI.getLeftX() * 0.8, 0.05),
                -MathUtil.applyDeadband(driverOI.getRightX() * 0.4, 0.05),
                true),
            swerveDrive));

    // DRIVER Left Trigger: While held, switch to virtual high gear.
    driverOI.leftTrigger()
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));


    // DRIVER Back Button: Set swerve drive to a stationary X position.
    driverOI.back().whileTrue(new RunCommand(() -> swerveDrive.setWheelsToXFormation(), swerveDrive));

    // DRIVER Start Button: Reset gyro to a new field oriented forward position.
    driverOI.start().whileTrue(new InstantCommand(() -> swerveDrive.resetGyro(), swerveDrive));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}