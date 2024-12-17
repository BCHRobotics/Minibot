package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
public Drivetrain(){
    // initialize intances fields in the constructor
    this.leftMotor = new CANSparkMax(deviceId:13, MotorType.kBrushless);
    this.rightMotor = new CANSparkMax(deviceId:11, MotorType.kBrushless);

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

        this differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        this.differentialDrive.setMaxOutput(setMaxOutput:0.25);
        this.differentialDrive.setSafetyEnabled(enabled:true);
        this.differentialDrive.setExpiration(expirationTime:0.1);

    }
// method for motor temperature
    public double[] motorTemperature(){
        double[] motorTemperature = [this.leftmotor.getMotorTemperature(), this.rightMotor.getMotorTemperature()];
        return motorTemperature;
        
    }
//method for motor voltage
    public double[] motorVoltage(){
        double[] motorVoltage = [this.leftMotor.getBusVoltage(), this.rightMotor.getBusVoltage()];
        return motorVoltage;
    }
// command for arcade drive
public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier turn) {
    return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble()turn.getAsDouble()));

        }
// command for brake
public Command Brake() {
    return runOnce(() -> {
        this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        } )
    }
//command for releasing brakes
public Command releaseBrakes( {
    return runOnce (() -> {
        this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.rightMotor.setIdleMOde(CANSparkMax.IdleMode.kCoast);
    })
})
// command for the slow arcade drive
public Command slowArcadeDriveCommand(DoubleSupplier slowForward, DoubleSupplier slowTurn) {
    return run(() -> this.differentialDrive.arcadeDrive(slowForward.getAsDouble() *0.5, slowTurn.getAsDouble() *0.5));
    }

}




