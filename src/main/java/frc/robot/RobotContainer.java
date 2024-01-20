package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  // Subsystems
  public final SwerveDrive swerveDrive;
  public final Hand hand;
  public final Arm arm;

  // Shuffleboard
  ShuffleboardTab rizzoTab;
  public static GenericEntry armEncoder;
  public static GenericEntry handLS;
  public static GenericEntry armLS;
  public static GenericEntry armTroubleShooting;

  // OI (Operator Interface) controllers
  public static CommandXboxController driverOI;
  public static CommandXboxController operatorOI;
  public static XboxController driverRumbler;
  public static XboxController operatorRumbler;

  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();
    hand = new Hand();
    arm = new Arm();

    // Configure the Shuffleboard
    rizzoTab = Shuffleboard.getTab("Rizzo");
    armEncoder = rizzoTab.add("Arm Encoder", -0.12345).getEntry();
    armLS = rizzoTab.add("Arm LS", false).getEntry();
    handLS = rizzoTab.add( "Hand LS", false).getEntry();
    armTroubleShooting = rizzoTab.add("ArmDirectControl", 0).getEntry();

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

    // DRIVER Right Trigger: While held, intake the hand
    driverOI.rightTrigger().whileTrue(new HandIntake(hand));

    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    driverOI.povUp().whileTrue(new NudgeRobot(swerveDrive, "NUDGE FORWARD").withTimeout(0.5));
    driverOI.povDown().whileTrue(new NudgeRobot(swerveDrive, "NUDGE BACKWARD").withTimeout(0.5));
    driverOI.povLeft().whileTrue(new NudgeRobot(swerveDrive, "NUDGE LEFT").withTimeout(0.5));
    driverOI.povRight().whileTrue(new NudgeRobot(swerveDrive, "NUDGE RIGHT").withTimeout(0.5));

    // DRIVER X,Y,B,A Buttons: Set chassis to predefined field relative angle.
    driverOI.x().whileTrue(new RotateRobot(swerveDrive, (double) -90));
    driverOI.y().whileTrue(new RotateRobot(swerveDrive, (double) 0));
    driverOI.b().whileTrue(new RotateRobot(swerveDrive, (double) 90));
    driverOI.a().whileTrue(new RotateRobot(swerveDrive, (double) 180));

    // DRIVER Back Button: Set swerve drive to a stationary X position.
    driverOI.back().whileTrue(new RunCommand(() -> swerveDrive.setWheelsToXFormation(), swerveDrive));

    // DRIVER Start Button: Reset gyro to a new field oriented forward position.
    driverOI.start().whileTrue(new InstantCommand(() -> swerveDrive.resetGyro(), swerveDrive));

    // Operator Right Trigger: While held, shoot from hand
    operatorOI.rightTrigger().whileTrue(new HandShoot(hand));

    // Operator Right Bumper: While held, outtake from hand
    operatorOI.rightBumper().whileTrue(new HandOuttake(hand));

    // OPERATOR Left Stick Y: Direct control over the Arm.
    new Trigger( () -> Math.abs( operatorOI.getLeftY() ) >= 0.05 ).whileTrue(
      new RunCommand( () -> arm.move( operatorOI.getLeftY() ), arm ) );

    // Operator DPAD Up, Right, Left, Down: Set Arm Target Positions
    // operatorOI.povUp().whileTrue(new ArmToAngle(arm, ArmConstants.ARM_HIGH_POS));
    // operatorOI.povLeft().whileTrue(new ArmToAngle(arm, ArmConstants.ARM_MED_POS));
    // operatorOI.povRight().whileTrue(new ArmToAngle(arm, ArmConstants.ARM_MED_POS));
    // operatorOI.povDown().whileTrue(new ArmToAngle(arm, ArmConstants.ARM_LOW_POS));

    arm.setDefaultCommand( new RunCommand( () -> arm.stop(), arm ) );
  }

  public Command getAutonomousCommand() {
    return new ParallelCommandGroup(
      new SequentialCommandGroup(
        new RunCommand( () -> hand.shoot(), hand).withTimeout(2.0),
        new RunCommand( () -> hand.stop(), hand) ),
      new SequentialCommandGroup(
        new WaitCommand(3.0),
        new RunCommand(
          () -> swerveDrive.drive(-1.0, 0, 0, true ), swerveDrive).withTimeout(2.5),
          new RunCommand(
            () -> swerveDrive.drive(0, 0, 0, true ), swerveDrive) ) );
  }
}