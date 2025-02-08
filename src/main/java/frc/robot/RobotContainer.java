package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.subsystems.Drivetrain;

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

    private final Elevator elevator = new Elevator();

    private final CommandXboxController driverController = new CommandXboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);
    

    public Elevator getElevator() {
        return elevator;
    }

    public RobotContainer() {
        configureButtonBindings();
    }
    
    //configuring button bindings 
    private void configureButtonBindings() {
        this.driverController.a()
        .onTrue(elevator.moveLevel("L1"));
        this.driverController.b()
        .onTrue(elevator.moveLevel("L2"));
        this.driverController.y()
        .onTrue(elevator.moveLevel("L3"));
        this.driverController.x()
        .onTrue(elevator.moveLevel("DOWN"));

        this.driverController.leftTrigger()
        .onTrue(elevator.moveLevel("L0"));
    }

 



}