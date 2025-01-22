package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.jni.CANSparkJNI;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

//import com.studica.frc.TitanQuad;
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


    private final DifferentialDrive differentialDrive;
    
    public Drivetrain() {
        this.left_topMotor = new TitanQuadMotor(Constants.CHASSIS.LEFT_TOP_MOTOR_ID);
        this.right_topMotor = new TitanQuadMotor(Constants.CHASSIS.LEFT_BOTTOM_MOTOR_ID);
        this.left_bottomMotor = new TitanQuadMotor(Constants.CHASSIS.RIGHT_TOP_MOTOR_ID);
        this.right_bottomMotor = new TitanQuadMotor(Constants.CHASSIS.RIGHT_BOTTOM_MOTOR_ID);
    
        left_topMotor.set(-0.5);
  
        left_topMotor.setInverted(true);
        right_bottomMotor.setInverted(true);
        
        left_bottomMotor.setInverted(false);
        right_topMotor.setInverted(false);

        this.left_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
        this.right_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
        this.left_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
        this.right_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
        
        
        //this.differentialDrive = new DifferentialDrive();
        
        this.differentialDrive.setMaxOutput(0.25); // Limit maximum speed
        this.differentialDrive.setSafetyEnabled(true); // Enable safety features
        this.differentialDrive.setExpiration(0.1); // Motor safety timeout
        
    }

    public Command brake() {
        return runOnce(() -> {
            this.left_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kBrake);
            this.right_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kBrake);
            this.left_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kBrake);
            this.right_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kBrake);
        
        });
    }

    public Command releaseBrakes() {
        return runOnce(() -> {
            this.left_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
            this.right_topMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
            this.left_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
            this.right_bottomMotor.setIdleMode(TitanQuadMotor.IdleMode.kCoast);
           
        });
    }

    public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier turn) {
        return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble()));
    }

    public Command driveForward(DoubleSupplier forward, DoubleSupplier turn) {
        return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble()));
    }

    private static class TitanQuadMotor {
        private final CAN motorCAN;
        private boolean isInverted = false;
    
        public TitanQuadMotor(int canId) {
            this.motorCAN = new CAN(canId);
        }

        public void set(double speed) {
            //left_topMotor.set(0.5);

        }
    }


}
