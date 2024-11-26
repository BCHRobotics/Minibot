package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;

public class AlignToHeadingCommand extends Command {
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    DoubleSupplier currentHeading;
    DoubleSupplier desiredHeading;

    public AlignToHeadingCommand(DoubleSupplier current, DoubleSupplier desired, Drivetrain subsystem) {
        // Assign the variables that point to input values
        currentHeading = current;
        desiredHeading = desired;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        driveSubsystem.setDriveMode(DriveMode.HEADINGLOCK);

        driveSubsystem.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND);
        driveSubsystem.enableRampRate();

        // Tell the driver that headinglock driving has been enabled
        System.out.println("HEADINGLOCK ON");
    }

    @Override
    public void execute() {
        // Turn the robot to a heading
        
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            System.out.println("HeadingLock Off: Interrupted");
        }
        else {
            System.out.println("HeadingLock Off: DriveMode");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not headinglock
        return driveSubsystem.getDriveMode() != DriveMode.HEADINGLOCK;
    }
}
