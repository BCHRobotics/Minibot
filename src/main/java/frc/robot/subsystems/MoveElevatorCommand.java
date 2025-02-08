package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveElevatorCommand extends CommandBase {
    private final Elevator elevator;
    private final double targetPosition;

    public MoveElevatorCommand(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotors();
    }
}
