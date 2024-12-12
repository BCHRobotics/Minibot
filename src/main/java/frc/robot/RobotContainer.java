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

    public class Robot extends TimedRobot {
        private RobotContainer robotContainer;
        
        public Drivetrain getDrivetrain() {
            return m_drivetrain;
        }
    }

    
    private final Drivetrain m_drivetrain = new Drivetrain();
    

    configureButtonBindings();
    configureDefaultCommands();

    private void configureButtonBindings() {

        this.driverController.leftBumper()
        .whileTrue(this.m_drivetrain.brake())
        .onFalse(this.m_drivetrain.releaseBrakes());

    }

    private void configureDefaultCommands() {
        
        Command drivingCommand = m_drivetrain.arcadeDriveCommand(-this.driverController.getLeftY(), -this.driverController.getRightX());

        m_drivetrain.setDefaultCommand(drivingCommand);

    }

    private final Drivetrain m_drivetrain = new Drivetrain();

    private final Command XboxController driverController = new CommandXboxController(Contants.CONTROLLER.DRIVER_CONTROLLER_PORT);


}
