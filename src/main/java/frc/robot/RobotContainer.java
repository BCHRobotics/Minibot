// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import required modules
import frc.robot.Commands.Autos;
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

// Import required libraries
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Mechanism mechanism = new Mechanism();

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(PERIPHERALS.DRIVER_PORT);
  CommandXboxController operatorController = new CommandXboxController(PERIPHERALS.OPERATOR_PORT);

  private final Command scoreGamePiece = Autos.automatedScoringCommand(drivetrain, mechanism);

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Set default commands

    // Control the drive with split-stick arcade controls
    this.drivetrain.setDefaultCommand(
        this.drivetrain.arcadeDriveCommand(
            () -> -this.driverController.getLeftY(), () -> -this.driverController.getRightX(),
            () -> this.driverController.getLeftTriggerAxis(), () -> this.driverController.getRightTriggerAxis()));

    //registering named commands for pathplanner, these can be set in the GUI using the same names
    NamedCommands.registerCommand("ARM LOW", this.mechanism.setArmPreset(MECHANISM.LOW));
    NamedCommands.registerCommand("ARM MID", this.mechanism.setArmPreset(MECHANISM.MID));
    NamedCommands.registerCommand("ARM HIGH", this.mechanism.setArmPreset(MECHANISM.HIGH));
    NamedCommands.registerCommand("ARM STOWED", this.mechanism.setArmPreset(MECHANISM.STOWED));

    NamedCommands.registerCommand("GRAB", this.mechanism.grabCone());
    NamedCommands.registerCommand("SHOOT", this.mechanism.launchGamePiece());
    NamedCommands.registerCommand("RELEASE", this.mechanism.releaseGamePiece());

    configureBindings();

    //building the auto chooser for pathplanner
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define bindings between conditions and commands. These are
   * useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>
   * Should be called during {@link Robot#robotInit()}.
   *
   * <p>
   * Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {

    // Driver braking and emergency stop controls
    this.driverController.rightBumper().or(this.driverController.y())
        .onTrue(this.drivetrain.enableBrakeMode()).onFalse(this.drivetrain.releaseBrakeMode());

    this.driverController.leftBumper()
        .whileTrue(this.drivetrain.enableBrakeMode()
            .andThen(this.drivetrain.emergencyStop()))
        .onFalse(this.drivetrain.releaseBrakeMode());

    // Driver automated routines
    this.driverController.a().whileTrue(this.drivetrain.seekTarget())
        .onFalse(Commands.runOnce(this.drivetrain::resetLimelight));
    this.driverController.x().whileTrue(this.drivetrain.goToTarget())
        .onFalse(Commands.runOnce(this.drivetrain::resetLimelight));
    this.driverController.y().whileTrue(this.drivetrain.balance());
    this.driverController.b().onTrue(Commands.runOnce(this.drivetrain::resetEncoders));
    this.driverController.povRight().onTrue(this.scoreGamePiece);
    this.driverController.povUp().onTrue(Autos.automatedScoringCommand(drivetrain, mechanism));

    // Operator arm preset controls

    this.operatorController.leftStick().onTrue(this.mechanism.setArmPreset(MECHANISM.HOME));
    this.operatorController.povLeft().onTrue(this.mechanism.setArmPreset(MECHANISM.STOWED));
    this.operatorController.povDown().onTrue(this.mechanism.setArmPreset(MECHANISM.LOW));
    this.operatorController.povRight().onTrue(this.mechanism.setArmPreset(MECHANISM.MID));
    this.operatorController.povUp().onTrue(this.mechanism.setArmPreset(MECHANISM.HIGH));
    this.operatorController.rightStick().onTrue(this.mechanism.setArmPreset(MECHANISM.STATION));

    // Operator intake claw controls
    this.operatorController.x().onTrue(this.mechanism.grabCube());
    this.operatorController.y().onTrue(this.mechanism.grabCone());
    this.operatorController.a().onTrue(this.mechanism.releaseGamePiece());
    this.operatorController.b().onTrue(this.mechanism.disableClaw());
    this.operatorController.rightTrigger().onTrue(this.mechanism.launchGamePiece());

    // Operator game piece signals
    this.operatorController.leftBumper().whileTrue(this.mechanism.blinkCubeLED())
        .onFalse(this.mechanism.setCubeLED(false));
    this.operatorController.rightBumper().whileTrue(this.mechanism.blinkConeLED())
        .onFalse(this.mechanism.setConeLED(false));
  }

  /**
   * HALTS all chassis motors
   */
  public void EMERGENCY_STOP() {
    this.drivetrain.killSwitch();
    this.mechanism.shutDown();
  }

  /**
   * Resets arm to default position
   */
  public Command ARM_RESET() {
    return this.mechanism.setArmPreset(MECHANISM.HOME);
  }

  /**
   * Resets chassis state
   */
  public Command CHASSIS_RESET() {
    return this.drivetrain.releaseBrakeMode();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /*
   * Resets the gyro heading of the robot
   */
  public void resetHeading() {
    this.drivetrain.resetGyro();
  }

  /*
   * Resets the position of the robot
   */
  public void resetPosition() {
    this.drivetrain.resetXY();
  }
}
