package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

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

    private final Drivetrain m_drivetrain = new Drivetrain();

    private final CommandXboxController driverController = new CommandXboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);

    public Drivetrain getDrivetrain() {
        return m_drivetrain;
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
    
        // put code that sets robot to slow mode when left trigger is held below

        // put code that drives robot forward when 'a' is held

       
    }


    

}