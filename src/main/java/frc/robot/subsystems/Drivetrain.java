package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** 
 * Class for the drivetrain subsystem (A BLUEPRINT to CREATE the drivetrain subystem, the ACTUAL subsystem will be created in RobotContainer.java) 
 * Creates and sets up motors, sensors, etc..
 * Has commands related to the drivetrain (ex. commands for driving, braking, etc.)
 * 
 */
public class Drivetrain extends SubsystemBase {
    
    private final CAN left_topMotor;
    private final CAN right_topMotor;
    private final CAN left_bottomMotor;
    private final CAN right_bottomMotor;
    private final DifferentialDrive differentialDrive;
    
        public Drivetrain() {
            this.left_topMotor = new CAN(Constants.CHASSIS.LEFT, MotorType.kBrushed);
            this.right_topMotor = new CAN(2, MotorType.kBrushed);
            this.left_bottomMotor = new CAN(3, MotorType.kBrushed);
            this.right_bottomMotor = new CAN(4, MotorType.kBrushed);

        /*
            this.left_topMotor = new PWM(Constants.CHASSIS.LEFT_MOTOR_ID, MotorType.kBrushed);
            this.right_topMotor = new PWM(Constants.CHASSIS.RIGHT_MOTOR_ID, MotorType.kBrushed);
            
            //this.left_topMotor.setIdleMode(PWMPWM.IdleMode.kCoast);
            //this.right_topMotor.setIdleMode(PWMPWM.IdleMode.kCoast);
            
            
            this.differentialDrive = new DifferentialDrive(left_topMotor, right_topMotor, left_bottomMotor, right_bottomMotor);
            */
    }

    public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier turn) {
        return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble()));
    }

    public Command driveForward(DoubleSupplier forward, DoubleSupplier turn) {
        return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble()));
    }


}
