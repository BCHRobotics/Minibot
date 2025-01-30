package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// import com.studica.frc.TitanQuad;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* 
 * Class for the drivetrain subsystem (A BLUEPRINT to CREATE the drivetrain subystem, the ACTUAL subsystem will be created in RobotContainer.java) 
 * Creates and sets up motors, sensors, etc..
 * Has commands related to the drivetrain (ex. commands for driving, braking, etc.)
 * 
 */
public class Drivetrain extends SubsystemBase {
    
    private final TitanQuadMotor left_topMotor;
    private final TitanQuadMotor right_topMotor;
    private final TitanQuadMotor left_bottomMotor;
    private final TitanQuadMotor right_bottomMotor;


    //private final DifferentialDrive differentialDrive;
    
    public Drivetrain() {
        this.left_topMotor = new TitanQuadMotor(Constants.CHASSIS.LEFT_TOP_MOTOR_ID);
        this.right_topMotor = new TitanQuadMotor(Constants.CHASSIS.LEFT_BOTTOM_MOTOR_ID);
        this.left_bottomMotor = new TitanQuadMotor(Constants.CHASSIS.RIGHT_TOP_MOTOR_ID);
        this.right_bottomMotor = new TitanQuadMotor(Constants.CHASSIS.RIGHT_BOTTOM_MOTOR_ID);
    }

    public void drive(double forward, double strafe, double rotation) {
        double leftTopSpeed = forward + strafe + rotation;
        double rightTopSpeed = forward - strafe - rotation;
        double leftBottomSpeed = forward - strafe + rotation;
        double rightBottomSpeed = forward + strafe - rotation;

        // Set the motor speeds without normalization
        this.left_topMotor.set(leftTopSpeed);
        this.right_topMotor.set(rightTopSpeed);
        this.left_bottomMotor.set(leftBottomSpeed);
        this.right_bottomMotor.set(rightBottomSpeed);
    }

    public Command brake() {
        return runOnce(() -> {
            this.left_topMotor.set(0);
            this.right_topMotor.set(0);
            this.left_bottomMotor.set(0);
            this.right_bottomMotor.set(0);
        
        });
    }

    public Command releaseBrakes() {
        return runOnce(()-> {
            this.left_topMotor.set(1);
            this.right_topMotor.set(1);
            this.left_topMotor.set(1);
            this.right_topMotor.set(1);
        });

    }

    /*
    public Command releaseBrakes() {
        return runOnce(() -> {
            this.left_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
            this.right_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
            this.left_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
            this.right_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
           
        });
    }
*/

    public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        return run(() -> this.drive(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble()));
    }
/*
    public Command driveForward(DoubleSupplier forward, DoubleSupplier turn) {
        return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble()));
    }
*/
    private static class TitanQuadMotor {
        private final CAN motorCAN;
        private boolean setInverted = false;
    
        public TitanQuadMotor(int canId) {
            this.motorCAN = new CAN(canId);
        }

        public void set(double speed) {
            //left_topMotor.set(0.5);

        }

    }

}