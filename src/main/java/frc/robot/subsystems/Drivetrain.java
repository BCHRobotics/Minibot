package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * Class for the drivetrain subsystem (A BLUEPRINT to CREATE the drivetrain subystem, the ACTUAL subsystem will be created in RobotContainer.java) 
 * Creates and sets up motors, sensors, etc..
 * Has commands related to the drivetrain (ex. commands for driving, braking, etc.)
 * 
 */
public class Drivetrain extends SubsystemBase {

    // creataes variables
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final DifferentialDrive differentialDrive;
    private double speedMultiplier;
    private double inversion;
    private boolean toggleTurnLock;
    private double turboMultiplier;
    private Timer timer;

    public Drivetrain() {
    // initialize instance fields in the constructor
        this.leftMotor = new CANSparkMax(13, MotorType.kBrushless);
        this.rightMotor = new CANSparkMax(11, MotorType.kBrushless);

        // Reset motor controllers to factory defaults
        this.leftMotor.restoreFactoryDefaults();
        this.rightMotor.restoreFactoryDefaults();
         
        // Set current limits for the motors to prevent overloading
        this.leftMotor.setSmartCurrentLimit(60, 20);
        this.rightMotor.setSmartCurrentLimit(60, 20);
    
        this.leftMotor.setInverted(true);
        this.rightMotor.setInverted(false);
    
        // Set motor modes (for braking, etc.)de
        this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        this.differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        this.differentialDrive.setMaxOutput(0.25);
        this.differentialDrive.setSafetyEnabled(true);
        this.differentialDrive.setExpiration(0.1);

        // defines the multipliers for the controls
        this.speedMultiplier = 1;
        this.inversion = 1;
        this.turboMultiplier = 1;
        this.timer = new Timer();
        this.timer.start();
    }

    // Main driving fuction takes a forward and turn value and reutrns a command that moves the robot accordinly
    public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier turn) {
            return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble() * speedMultiplier * inversion * turboMultiplier, turn.getAsDouble() * (toggleTurnLock ? 0 : 1) * speedMultiplier * inversion * turboMultiplier));
        }
    
    // Alternate driving function that returns a command to move the robot only forward
    public Command driveForward(double forward) {
        return run(() -> this.differentialDrive.arcadeDrive(forward * speedMultiplier * turboMultiplier,0));

    }

    // gets a multiplier for the speed of the robot that is then applied to the main driving method
    public Command setSpeedMultiplier(double multiplier) {

        return runOnce(() -> {
            speedMultiplier = multiplier;
        });
    }

    public Command setturboMultiplier(double multiplier) {

        return runOnce(() -> {
            turboMultiplier = multiplier;
        });

    }

    // funtion that inverts the controls (multiplies everything by -1) (bonus thing, not part of assignment)
    public Command setInversion(double multiplier) {

        return runOnce(() -> {
            inversion = multiplier;
        });
    }

    // funtion that returns a toggable lock for the turning (bonus)
    public Command toggleTurnLock() {

        return runOnce(() -> {
            toggleTurnLock = !toggleTurnLock;
        });

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

    public void printMotorStats() {
        
        if (this.timer.get() >= 2.0) {
        
        System.out.println("The left motor temprature is " + leftMotor.getMotorTemperature() + ".");
        System.out.println("The right motor temprature is " + rightMotor.getMotorTemperature() + ".");
       
        // SmartDashboard.putNumber("Left motor voltage", leftMotor.getBusVoltage());
       
        System.out.println("The left motor voltage is " + leftMotor.getBusVoltage() + ".");
        System.out.println("The right motor voltage is " + rightMotor.getBusVoltage() + ".");

        this.timer.reset();
    }}

}