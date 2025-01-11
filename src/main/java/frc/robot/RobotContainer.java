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

    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Elevator m_elevator = new Elevator();

    private final CommandXboxController driverController = new CommandXboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController driverController2 = new CommandXboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT2);
    // importing drivetrain
    public Drivetrain getDriveTrain(){
        return m_drivetrain;
    }

    public RobotContainer() {
        //calling method names
        configureButtonBindings();
        configureDefaultCommands();
        configureTriggerBindings();
        configureDriveForwardA();
        
    }

        // button bindings for using left bumper it will brake if pressed

        private void configureButtonBindings(){
    
            this.driverController.leftBumper()
            .whileTrue(this.m_drivetrain.brake())  // brakes on when button is held
            .onFalse(this.m_drivetrain.releaseBrakes()); 
            
            this.driverController2.rightBumper()
            .whileTrue(this.m_elevator.stop())
            .onFalse(this.m_elevator.releaseStop());
            // brakes off when button is released
    
        }
        // arcade drive
        private void configureDefaultCommands(){
            Command drivingCommand = m_drivetrain.arcadeDriveCommand(() -> -this.driverController.getLeftY(), () -> -this.driverController.getRightX());
            // set the default behavior of the drive subsystems to react to joystick inputs
            m_drivetrain.setDefaultCommand(drivingCommand);
        }
        
        // method for trigger drive
        private void configureTriggerBindings(){
            // setting the left trigger to drive slowly
            Command triggerDrive = m_drivetrain.turboMode();
            // if left trigger is being pressed or true it will drive slowly if false it will coast
            this.driverController.leftTrigger()
            .whileTrue(triggerDrive)
            .onFalse(this.m_drivetrain.normalMode());
        }

        //driving forward with A command
        private void configureDriveForwardA(){
    
            //setting the command for A drive
            Command aDrive = m_drivetrain.arcadeDriveCommand(() -> 0.25, () -> 0);
            //saying that if the driver controller button a is being pressed if true drive forward if false coast
            this.driverController.a()
            .whileTrue(aDrive);
            
        }

        }


