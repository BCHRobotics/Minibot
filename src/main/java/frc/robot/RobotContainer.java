// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.AlignToHeadingCommand;
// Import required modules
import frc.robot.Commands.Autos;
import frc.robot.Commands.TeleopDriveCommand;
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.DoubleSubscriber;
// Import required libraries
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(PERIPHERALS.DRIVER_PORT);

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Set default commands

    DoubleSupplier yCommand = () -> adjustJoystickInput(() -> -this.driverController.getLeftY(), 0.4);
    DoubleSupplier xCommand = () -> adjustJoystickInput(() -> -this.driverController.getRightX(), 0.4);

    //Control the drive with split-stick arcade controls
    this.drivetrain.setDefaultCommand(
        new TeleopDriveCommand(
            yCommand, xCommand,
            () -> this.driverController.getLeftTriggerAxis(), () -> this.driverController.getRightTriggerAxis(), drivetrain));

    // this.drivetrain.setDefaultCommand(
    //     this.drivetrain.alignToHeadingCommand(
    //         () -> this.drivetrain.getHeadingDeg(), () -> this.drivetrain.getDesiredHeading()));

    configureBindings();
    configureNamedCommands();

    //building the auto chooser for pathplanner
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /*
   * Used for confirming named commands for pathplanner
   */
  public void configureNamedCommands() {
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

    // [Test] Align to heading of 90 when right bumper is pressed
    // this.driverController.rightBumper().onTrue(
    //   new AlignToHeadingCommand(null, null, null, null, drivetrain));
  }

  public double adjustJoystickInput(DoubleSupplier input, double scale) {
      return input.getAsDouble() * scale;
  }

  /**
   * HALTS all chassis motors
   */
  public void EMERGENCY_STOP() {
    this.drivetrain.killSwitch();
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
   * Resets the field relative position of the robot
   */
  public void resetPosition() {
    this.drivetrain.resetFieldPosition();
  }
}
