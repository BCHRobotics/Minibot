package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

public class RobotContainer {
    private final Elevator elevator = new Elevator();
    private final XboxController controller = new XboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Example button bindings for elevator control
        new Trigger(() -> controller.getAButton())
            .onTrue(new MoveElevatorCommand(elevator, ElevatorConstants.L1));

        new Trigger(() -> controller.getBButton())
            .onTrue(new MoveElevatorCommand(elevator, ElevatorConstants.L2));

        new Trigger(() -> controller.getXButton())
            .onTrue(new MoveElevatorCommand(elevator, ElevatorConstants.L3));

        new Trigger(() -> controller.getYButton())
            .onTrue(new MoveElevatorCommand(elevator, ElevatorConstants.bottomPos));
    }

    public Elevator getElevator() {
        return elevator;
    }
}
