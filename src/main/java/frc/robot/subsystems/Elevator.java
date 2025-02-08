package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private double setpoint = ElevatorConstants.bottomPos;
    private final double upSpeed = 0.6; //Sets the up speed
    private final double downSpeed = -0.5; //Sets the down speed


    public Elevator() {
        primaryMotor = new SparkMax(ElevatorConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.followerMotor, MotorType.kBrushless);

        // Configure follower motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);
        followerMotor.configure(followerConfig, null, null);

        encoder = primaryMotor.getEncoder();
        topLimit = new DigitalInput(ElevatorConstants.topLimitSwitchPort);
        bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitSwitchPort);

        // Motor safety configuration
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(40);
        motorConfig.voltageCompensation(12.0);
        
        primaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
    }


    public void setTargetPosition(double positionInches) {
        setpoint = MathUtil.clamp(positionInches, ElevatorConstants.bottomPos, ElevatorConstants.topPos);
    }

    public void stopMotors() {
        primaryMotor.set(0);
    }

    public void run() {
        double currentPosition = encoder.getPosition() / ElevatorConstants.countsPerInch;

        if (!bottomLimit.get()) {  // If bottom limit switch is pressed
            stopMotors();
            encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch); // Reset encoder
            setpoint = ElevatorConstants.bottomPos;
            return;
        }

        // Handle top limit switch
        if (!topLimit.get()) {  // If top limit switch is pressed
            stopMotors();
            encoder.setPosition(ElevatorConstants.topPos * ElevatorConstants.countsPerInch); // Reset encoder
            setpoint = ElevatorConstants.topPos;
            return;}
        
            if (currentPosition < setpoint) {
                primaryMotor.set(upSpeed); // Moves up
            } else if (currentPosition > setpoint) {
                primaryMotor.set(downSpeed); // Moves down
            } else {
                stopMotors(); // Stop when setpoint is reached
            }
    
            // Display data on dashboard
            SmartDashboard.putNumber("Elevator Position", currentPosition);
            SmartDashboard.putNumber("Elevator Setpoint", setpoint);
            SmartDashboard.putBoolean("Top Limit Reached", !topLimit.get());
            SmartDashboard.putBoolean("Bottom Limit Reached", !bottomLimit.get());
        }
    }
