package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final DifferentialDrive differentialDrive; 

    public Drivetrain() {

        this.leftMotor = new CANSparkMax(Constants.CHASSIS.LEFT_MOTOR_ID, MotorType.kBrushless);
        this.rightMotor = new CANSparkMax(Constants.CHASSIS.RIGHT_MOTOR_ID, MotorType.kBrushless);

        this.leftMotor.restoreFactoryDefaults();
        this.rightMotor.restoreFactoryDefaults();

        // Set current limits for the motors to prevent overloading
        this.leftMotor.setSmartCurrentLimit(60, 20);
        this.rightMotor.setSmartCurrentLimit(60, 20);

        this.leftMotor.setInverted(true);
        this.rightMotor.setInverted(false);

        // Set motor modes (for braking, etc.)
        this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        this.differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        this.differentialDrive.setMaxOutput(0.25);
        this.differentialDrive.setSafetyEnabled(true);
        this.differentialDrive.setExpiration(0.1);
    }

    private double speedMultiplier = 1;
    public Command slowMode() {
        return runOnce(() -> speedMultiplier = 0.5);
    }


    public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier turn) {
        return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble() * speedMultiplier, turn.getAsDouble()*speedMultiplier));
    }

    //drive forward when pressing a
    public Command driveForward(DoubleSupplier forward, DoubleSupplier turn) {
        return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble()));
    }

    public Command brake() {
        return runOnce(() -> {
            this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);    
        });
    }

    public Command releaseBrakes() {
        return runOnce(() -> {
            this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        });

    }

    public void printTemp() {
        //getting motor temperature
        System.out.println(this.leftMotor.getMotorTemperature());
        System.out.println(this.rightMotor.getMotorTemperature());
        //get voltage
        System.out.println(this.leftMotor.getBusVoltage());
        System.out.println(this.rightMotor.getBusVoltage());

    }

}