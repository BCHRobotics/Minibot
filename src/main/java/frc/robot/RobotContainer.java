package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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

    private void configureButtonBindings() {
        
        this.driverController.leftBumper()
        .whileTrue(this.m_drivetrain.brake())
        .onFalse(this.m_drivetrain.releaseBrakes());

        this.driverController.rightBumper()
        .whileTrue(this.m_drivetrain.slowMode())
        .onFalse(this.m_drivetrain.normalMode());
    }

    private void configureDefaultCommands() {
        Command drivingCommand = m_drivetrain.arcadeDriveCommand(
            () -> -this.driverController.getLeftX(),
            () -> -this.driverController.getRightX()
        );
            m_drivetrain.setDefaultCommand(drivingCommand);


        
    }


    
    }



