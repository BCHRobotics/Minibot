package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final CANSparkMax primaryMotor;
    private final CANSparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;

    private double setpoint = ElevatorConstants.downPos;

    public Elevator() {
        primaryMotor = new CANSparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);

        followerMotor.follow(primaryMotor, true);

        // Configure motors for safety and performance
        primaryMotor.restoreFactoryDefaults();
        primaryMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        primaryMotor.setSmartCurrentLimit(40);
        primaryMotor.enableVoltageCompensation(12.0);

        followerMotor.restoreFactoryDefaults();
        followerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        followerMotor.setSmartCurrentLimit(40);
        followerMotor.enableVoltageCompensation(12.0);

        // Set soft limits
        primaryMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ElevatorConstants.topPos * (float) ElevatorConstants.countsPerInch);
        primaryMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ElevatorConstants.bottomPos * (float) ElevatorConstants.countsPerInch);
        primaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        primaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        pidController = new PIDController(
            ElevatorConstants.ElevatorkP,
            ElevatorConstants.ElevatorkI,
            ElevatorConstants.ElevatorkD
        );
        pidController.setTolerance(0.5);
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

    @Override
    public void periodic() {
        double currentPosition = encoder.getPosition() / ElevatorConstants.countsPerInch;

        double pidOutput = pidController.calculate(currentPosition, setpoint);
        double feedForward = ElevatorConstants.feedForward * Math.signum(setpoint - currentPosition);
        double output = pidOutput + feedForward;

        output = MathUtil.clamp(output, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
        primaryMotor.set(output);

        handleBottomLimit();

        SmartDashboard.putNumber("Elevator Position", currentPosition);
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator PID Output", output);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
}
