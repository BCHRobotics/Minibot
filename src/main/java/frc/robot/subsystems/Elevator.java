package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Elevator extends SubsystemBase {
    private final CANSparkMax elevatorLeftMotor;
    private final CANSparkMax elevatorRightMotor;






public Elevator() {
    this.elevatorLeftMotor = new CANSparkMax(13, MotorType.kBrushless);
    this.elevatorRightMotor = new CANSparkMax(11, MotorType.kBrushless);

    this.elevatorLeftMotor.restoreFactoryDefaults();
    this.elevatorRightMotor.restoreFactoryDefaults();
     
    // Set current limits for the motors to prevent overloading
    this.elevatorLeftMotor.setSmartCurrentLimit(60, 20);
    this.elevatorRightMotor.setSmartCurrentLimit(60, 20);

    this.elevatorLeftMotor.setInverted(true);
    this.elevatorRightMotor.setInverted(false);

    // Set motor modes (for braking, etc.)
    this.elevatorLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    this.elevatorRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);


    
}



public Command moveUp(DoubleSupplier forward, DoubleSupplier turn) {
    return run(() -> this.(forward.getAsDouble(), turn.getAsDouble()));
    
        }

public Command stop() {
    return runOnce(() -> {
        this.elevatorLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.elevatorRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        });
    }


public Command releaseStop() {
    return runOnce (() -> {
        this.elevatorLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.elevatorRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    });
}


}