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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase{

    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;
    // private final TrapezoidProfile.Constraints constraints;
    // private TrapezoidProfile.State goalState;
    // private TrapezoidProfile.State currentState;
    // private final TrapezoidProfile profile;

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    private double setpoint = 0.0;
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;

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
 
    public Elevator() {
        
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);

        //configure motors
        followerMotor.configure(followerConfig, null, null); 
        
        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);
        
        //setting limits for safety
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);

        pidController = new PIDController(
            ElevatorConstants.ElevatorkP,
            ElevatorConstants.ElevatorkI,
            ElevatorConstants.ElevatorkD
        );
        
        pidController.setTolerance(0.5);

        configureMotors();
    }
    
    private void configureMotors() {
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }
    
    private void handleBottomLimit() {
        stopMotors();
        encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch);
        isHomed = true;
        setpoint = ElevatorConstants.bottomPos;
        //currentState = new TrapezoidProfile.State(ElevatorConstants.bottomPos, 0);
        //goalState = new TrapezoidProfile.State(ElevatorConstants.bottomPos, 0);
        pidController.reset();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }


    public void homeElevator() {
        primaryMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        if (bottomLimit.get()) {
            handleBottomLimit();
        }
    }

    public void run() {
        currentPos = encoder.getPosition() / ElevatorConstants.countsPerInch;
        /* 
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        if (bottomLimit.get()) {
            handleBottomLimit();
        }

        if (getHeightInches() > ElevatorConstants.maxPos) {
            stopMotors();
        }
        
        double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
        double ff = calculateFeedForward(currentState);
        
        double outputPower = MathUtil.clamp(
            pidOutput + ff,
            -ElevatorConstants.max_output,
            ElevatorConstants.max_output   
            );     
            
            primaryMotor.set(outputPower);
    */
        }
}

