package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        //Initialize instance fields in the constructor
        this.leftMotor = new CANSparkMax (13, MotorType.kBrushless);
        this.rightMotor = new CANSparkMax(11, MotorType.kBrushless);
        this.differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        this.differentialDrive.setMaxOutput(0.25);
        this.differentialDrive.setSafetyEnabled(true);
        this.differentialDrive.setExpiration(0.1);
    
    
        // Reset motor controllers to factory defaults
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

    }

    public Command brake() {
        return runOnce(() -> {
            this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        });
    }
    
    public Command releaseBrakes() {
        return runOnce(() -> {
            this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        });
    }
    
    private double speedMultiplier = 1;
    public Command arcadeDriveCommand (DoubleSupplier forward, DoubleSupplier turn) {
        return run (() -> this.differentialDrive.arcadeDrive(forward.getAsDouble() * speedMultiplier, turn.getAsDouble() * speedMultiplier));//Multiplies the forward and turn speed by the speed multiplier
    }
    
    public Command slowmode() {
        return runOnce (() -> speedMultiplier=0.5 );//Makes the speed multiplier 0.5
    }

    public Command normalmode() {
        return runOnce (() -> speedMultiplier=1);//Makes the speed multiplier 1
    }
    
    public Command driveForward () { 
        return run (() -> this.differentialDrive.arcadeDrive( 0.25,  0 ));//sets the forward speed to 0.25 and the turn speed to 0
    }
    
    public void getTemperature () {
        System.out.println(this.leftMotor.getMotorTemperature());//Gets the temperature of left motor
        System.out.println(this.rightMotor.getMotorTemperature());//Gets the temperature of the right motor
    }
    
    public void getBusVoltage () {
        System.out.println(this.leftMotor.getBusVoltage()); //Gets voltage of left motor
        System.out.println(this.rightMotor.getBusVoltage());//Gets the voltage of right motor
    }
}