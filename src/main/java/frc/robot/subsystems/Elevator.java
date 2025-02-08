package frc.robot.subsystems;
//import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

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
    
    //SparkMaxConfig resetConfig = new SparkMaxConfig();
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
        

        //setting limits for safety
        SparkMaxConfig resetConfig = new SparkMaxConfig();
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
            

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        //primaryMotor.setInverted(true);
        followerConfig.follow(primaryMotor, true);

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
        
    }
    
    private void handleBottomLimit() {
        if(!bottomLimit.get()) {
            stopMotors();
            encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch);
            setpoint = ElevatorConstants.bottomPos;
            
            if (setpoint == ElevatorConstants.bottomPos) {
                pidController.reset();
        }
    }
    } 
    
    public void setTargetPosition(double positionInches) {
        /* 
        if (!bottomLimit.get()) {
            setpoint = ElevatorConstants.bottomPos;
        } else { }
         */
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
        
        double pidOutput;
        //if (setpoint == ElevatorConstants.bottomPos) {
        pidOutput = pidController.calculate(currentPosition, setpoint);
        //} else {
        //    pidOutput = pidController.calculate(currentPosition, setpoint);
        //}
        double feedForward = ElevatorConstants.feedForward*Math.signum(setpoint-currentPosition);
        double output = pidOutput + feedForward;

        output = MathUtil.clamp(output, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
        
        primaryMotor.set(output);

        handleBottomLimit();

        System.out.println("Elevator Position" + currentPosition);
        System.out.println("Elevator Setpoint" +  setpoint);
        System.out.println("Elevator PID Output" + output);
        System.out.println("Elevator error" + (setpoint - currentPosition));
    }

    
    public Command moveLevel(String level) {
        return runOnce(()-> {
            switch (level) {
                case "L1":
                    setTargetPosition(ElevatorConstants.L1);
                    break;
                
                case "L2":
                    setTargetPosition(ElevatorConstants.L2);
                    break;
                
                case "L3":
                    setTargetPosition(ElevatorConstants.L3);
                    break;
                case "DOWN":
                    setTargetPosition(ElevatorConstants.bottomPos);
                    break;
            }
        });//.andThen(() -> stopMotors());
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

}

