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

    public RobotContainer() {

        configureButtonBindings();
        configureDefaultCommands();

    }
        private void configureButtonBindings() {
 
        }

        private void configureDefaultCommands() {

            this.driverController.leftBumper()
            .whileTrue(this.m_drivetrain.brake()) // Brakes on when button is held
            .onFalse(this.m_drivetrain.releaseBrakes()); // Brakes off when button isn't held

            Command drivingCommand = m_drivetrain.arcadeDriveCommand(
            () -> -this.driverController.getLeftY(), // Lambda (->) gets the current Y value
            () -> -this.driverController.getRightX() // Lambda (->) gets the current X value
        );

            // Set the default behavior of the drive subsystem to react to joystick inputs
            m_drivetrain.setDefaultCommand(drivingCommand);
            
            // gets the input for the a button and gives it to the drivetrain subsystem
            this.driverController.a()
            .whileTrue(this.m_drivetrain.driveForward(1)); 

            // gets the input for the left trigger button and gives it to the drivetrain subsystem
            this.driverController.leftTrigger()
            .onTrue(this.m_drivetrain.setSpeedMultiplier(0.75))
            .onFalse(this.m_drivetrain.setSpeedMultiplier(1));

            // swtiches the boolean variable to lock turning on the robot
            this.driverController.rightTrigger()
            .onTrue(this.m_drivetrain.toggleTurnLock());

            // set the inversion multiplier to -1 inverting the controls
            this.driverController.rightBumper()
            .onTrue(this.m_drivetrain.setInversion(-1))
            .onFalse(this.m_drivetrain.setInversion(1));

            this.driverController.x()
            .onTrue(this.m_drivetrain.setturboMultiplier(1))
            .onFalse(this.m_drivetrain.setturboMultiplier(0.75));
        }


    public Drivetrain getDrivetrain() {
        return m_drivetrain;

    }

    }