package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;

import frc.robot.Constants.ElevatorConstants;

public class RobotContainer {

    private final Elevator elevator = new Elevator();

    private final CommandJoystick joystick = new CommandJoystick(0);

    public RobotContainer() {
        configureBindings();

        // Default command for the elevator to maintain its position
        elevator.setDefaultCommand(Commands.run(elevator::run, elevator));
    }

    private void configureBindings() {
        // Bind joystick buttons for elevator control
        joystick.button(1).onTrue(Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L1), elevator));
        joystick.button(2).onTrue(Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L2), elevator));
        joystick.button(3).onTrue(Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L3), elevator));
        joystick.button(4).onTrue(Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.bottomPos), elevator));
    }

    public Command getAutonomousCommand() {
        return Commands.runOnce(() -> elevator.setTargetPosition(ElevatorConstants.L1));
    }
}
