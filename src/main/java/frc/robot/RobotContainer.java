package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

/** 
 * The RobotContainer is where we set up everything for the robot: 
 * 
 * - Creating Subsystem Objects (ex. drivetrain subsystem, elevator subystem) 
 *         NOTE: Jerry only has drivetrain subsystem 
 * 
 * - Creating controller for robot (xbox controller), and getting it's joystick values
 * 
 * - Button mappings (what buttons do what, ex. left bumper --> brake robot)
 * 
 */

public class RobotContainer {
    private final Elevator m_elevator = new Elevator();
    
    private final Drivetrain m_drivetrain = new Drivetrain();

    private final CommandXboxController driverController = new CommandXboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);

    public Drivetrain getDrivetrain() {
        return m_drivetrain;
    }
    public Elevator getElevator() {
        return m_elevator;
    }
    public RobotContainer() {

        configureButtonBindings();
        configureDefaultCommands();
    } 
    private void configureDefaultCommands() {
            Command drivingCommand = m_drivetrain.arcadeDriveCommand(
    () -> -this.driverController.getLeftY(), //Lambda (->) gets the current Y value
    () -> -this.driverController.getRightX() //Lambda (->) gets the current X value
);
    //Set the default behaviour of the drive subsystem to react to joystick inputs
    m_drivetrain.setDefaultCommand(drivingCommand);

    }

    private void configureButtonBindings() {

        this.driverController.leftBumper()
        .whileTrue(this.m_drivetrain.brake()) //Brakes on the button is held
        .onFalse(this.m_drivetrain.releaseBrakes()); //Brakes off when button is released
    
        this.driverController.a().whileTrue(this.m_drivetrain.driveForward());//Robot moves forward when the 'a' key is pressed
        this.driverController.leftTrigger().whileTrue(this.m_drivetrain.slowmode())
        .onFalse(this.m_drivetrain.normalmode());//Puts the robot into slow mode when the left trigger is pressed and goes back into normal mode when it is released
        this.driverController.rightTrigger().whileTrue(this.m_drivetrain.turbomode())
        .onFalse(this.m_drivetrain.normalmode());//Puts the robot into turbo mode when the right trigger is pressed and goes back into normal mode when it is released
    }
}
