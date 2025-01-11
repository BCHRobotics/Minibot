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
    private final CommandXboxConntroller driverController = new CommandXboxController(Contants.CONTROLLER.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {

        configureButtonBindings();
        configureDefaultCommands();
            
            }
        
            private void configureDefaultCommands() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'configureDefaultCommands'");
            }
        
            private void configureButtonBindings(){

    }

    private void configureButtonBindings(){

        this.driverController.leftBumper()
        .whileTure(this.m_drivetrain.brake());
        .onFalse(this.m_drivetrain.releaseBrakes());
    } 


    Command drivingCommand = m_drivetrain.arcadeDriveCommand(
    () -> -this.driverController.getLeftY(),
    () -> -this.driverController.getRightX(),
);

    m_drivetrain.setDefaultCommand(drivingCommand);

    public class Robot extends TimedRobot {
 
        private RobotContainer robotContainer;
    }

    public Drivetrain getDrivtrain() {
        return m_drivetrain;
    }
}




