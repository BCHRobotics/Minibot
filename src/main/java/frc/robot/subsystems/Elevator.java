package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;

    private double setpoint = ElevatorConstants.bottomPos;

    public Elevator() {
        primaryMotor = new SparkMax(ElevatorConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.followerMotor, MotorType.kBrushless);

        // Configure follower motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);
        followerMotor.configure(followerConfig, null, null);

        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);

        pidController = new PIDController(
            ElevatorConstants.ElevatorkP,
            ElevatorConstants.ElevatorkI,
            ElevatorConstants.ElevatorkD
        );
        pidController.setTolerance(0.5);

        // Motor safety configuration
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(40);
        motorConfig.voltageCompensation(12.0);
        
        primaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
    }

    private void handleBottomLimit() {
        if (!bottomLimit.get()) {
            stopMotors();
            encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch);
            setpoint = ElevatorConstants.bottomPos;
            pidController.reset();
        }
    }

    public void setTargetPosition(double positionInches) {
        setpoint = MathUtil.clamp(positionInches, ElevatorConstants.bottomPos, ElevatorConstants.topPos);
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public void run() {
        double currentPosition = encoder.getPosition() / ElevatorConstants.countsPerInch;

        handleBottomLimit();

        double pidOutput = pidController.calculate(currentPosition, setpoint);
        double feedForward = ElevatorConstants.feedForward * Math.signum(setpoint - currentPosition);
        double output = pidOutput + feedForward;

        output = MathUtil.clamp(output, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
        primaryMotor.set(output);

        SmartDashboard.putNumber("Elevator Position", currentPosition);
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator PID Output", output);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
}
