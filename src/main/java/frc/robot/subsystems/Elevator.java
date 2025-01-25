package frc.robot.subsystems;
//import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase{

    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;

    private double setpoint = 0;

    //private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;
/* 
    public enum ElevatorPosition {
        DOWN(ElevatorConstants.downPos),
        POSITION_1(ElevatorConstants.L1),
        POSITION_2(ElevatorConstants.L2),
        POSITION_3(ElevatorConstants.L3);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }
 */
    public Elevator() {
        
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);

        //configure motors
        followerMotor.configure(followerConfig, null, null); 
        
        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);
        
        
        pidController = new PIDController(
            ElevatorConstants.ElevatorkP,
            ElevatorConstants.ElevatorkI,
            ElevatorConstants.ElevatorkD
        );
         
        pidController.setTolerance(0.5);
        
        //setting limits for safety
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        
        configureMotors();
    }
    
    private void configureMotors() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);
        config.voltageCompensation(12.0);
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }
    
    private void handleBottomLimit() {
        if(!bottomLimit.get()) {
            stopMotors();
            encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch);
            setpoint = ElevatorConstants.bottomPos;
            pidController.reset();
        }
    } 
    
    public void setTargetPosition(double positionInches) {
        setpoint = MathUtil.clamp(
            positionInches, 
            ElevatorConstants.bottomPos, 
            ElevatorConstants.topPos);
    }


    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }


    public void run() {
        double currentPosition = encoder.getPosition() / ElevatorConstants.countsPerInch;

        handleBottomLimit();

        double pidOutput = pidController.calculate(currentPosition, setpoint);

        double feedForward = ElevatorConstants.feedForward*Math.signum(setpoint-currentPosition);
        double output = pidOutput + feedForward;

        primaryMotor.set(output);

        SmartDashboard.putNumber("Elevator Position", currentPosition);
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator PID Outpput", output);
    }

}

