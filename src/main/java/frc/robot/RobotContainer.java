// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.TeleopDriveCommand;
import frc.robot.Commands.Pathing.AlignToHeadingCommand;
import frc.robot.Commands.Pathing.AlignToPointCommand;
import frc.robot.Commands.Pathing.DriveStraightCommand;
import frc.robot.Commands.Pathing.FollowPathCommand;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoUtils;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(PERIPHERALS.DRIVER_PORT);


  public RobotContainer() {
    // Set default commands

    configureDefaultCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData(key: "auto Chooser", autoChooser);

    configureBindings();
    configureDefaultCommands();

      }
  
  public void configureDefaultCommands() {

  }

  
 
  public void configureBindings() {

  }

  /**
   * Multiplies a joystick input by a scale factor, and returns the result
   * @param input The joystick input
   * @param scale The multiplier
   * @return The scaled joystick input
   */
  public double adjustJoystickInput(DoubleSupplier input, double scale) {
      return input.getAsDouble() * scale;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return autoChooser.getSelected();

    // EVERYTHING FROM HERE ON DOWN IS A TEMPORARY TEST

    // // define the auto as a set of paths in a string
    // String commandString = "1";
    // // split up the command string and make an auto with it
    // return AutoUtils.BuildAutoFromCommands(AutoUtils.SeparateCommandString(commandString), drivetrain);

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
