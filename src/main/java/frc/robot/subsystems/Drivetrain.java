// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.MISC;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.Constants.VISION;
import frc.robot.Constants.VISION.TARGET_TYPE;
import frc.robot.util.control.SparkMaxPID;
import frc.robot.util.devices.Gyro;
import frc.robot.util.devices.Limelight;

public class Drivetrain extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final CANSparkMax frontLeftMotor;
  private final CANSparkMax frontRightMotor;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final SparkMaxPID leftMotorController;
  private final SparkMaxPID rightMotorController;

  private final DifferentialDrive drive;

  private final Gyro gyro;
  private final Limelight limelight;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final DifferentialDriveOdometry odometry;

  /** Creates a new Drive subsystem. */
  public Drivetrain() {

    this.frontLeftMotor = new CANSparkMax(CHASSIS.FRONT_LEFT_ID, MotorType.kBrushless);
    this.frontRightMotor = new CANSparkMax(CHASSIS.FRONT_RIGHT_ID, MotorType.kBrushless);

    this.frontLeftMotor.restoreFactoryDefaults();
    this.frontRightMotor.restoreFactoryDefaults();

    this.frontLeftMotor.setIdleMode(IdleMode.kCoast);
    this.frontRightMotor.setIdleMode(IdleMode.kCoast);

    this.frontLeftMotor.setSmartCurrentLimit(60, 20);
    this.frontRightMotor.setSmartCurrentLimit(60, 20);

    this.frontLeftMotor.setInverted(CHASSIS.INVERTED);
    this.frontRightMotor.setInverted(!CHASSIS.INVERTED);

    this.leftEncoder = this.frontLeftMotor.getEncoder();
    this.rightEncoder = this.frontRightMotor.getEncoder();

    this.leftEncoder.setPositionConversionFactor(CHASSIS.LEFT_POSITION_CONVERSION);
    this.rightEncoder.setPositionConversionFactor(CHASSIS.RIGHT_POSITION_CONVERSION);

    this.leftEncoder.setVelocityConversionFactor(CHASSIS.LEFT_VELOCITY_CONVERSION);
    this.rightEncoder.setVelocityConversionFactor(CHASSIS.RIGHT_VELOCITY_CONVERSION);

    this.leftMotorController = new SparkMaxPID(this.frontLeftMotor, CHASSIS.LEFT_DRIVE_CONSTANTS);
    this.rightMotorController = new SparkMaxPID(this.frontRightMotor, CHASSIS.RIGHT_DRIVE_CONSTANTS);

    this.leftMotorController.setFeedbackDevice(this.leftEncoder);
    this.rightMotorController.setFeedbackDevice(this.rightEncoder);

    this.leftMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);
    this.rightMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);

    this.gyro = Gyro.getInstance();
    this.limelight = Limelight.getInstance();
    this.limelight.setDesiredTarget(TARGET_TYPE.REFLECTIVE_TAPE);
    this.limelight.setLedMode(1);

    this.odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
    

    this.drive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);

    AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::setChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   * @param min the commanded snail percentage
   * @param max the commanded turbo percentage
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot,
      DoubleSupplier min, DoubleSupplier max) {
    return runOnce(() -> {
      this.setMaxOutput(CHASSIS.DEFAULT_OUTPUT + (max.getAsDouble() * CHASSIS.MAX_INTERVAL)
          - (min.getAsDouble() * CHASSIS.MIN_INTERVAL));
      this.arcadeDrive(fwd.getAsDouble(), (rot.getAsDouble() * 0.9));
    })
        .beforeStarting(() -> this.drive.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND))
        .beforeStarting(this.enableRampRate())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("arcadeDrive");
  }

  /**
   * Sets drivetrain position in inches
   */
  public Command positionDriveCommand(double leftPos, double rightPos) {
    return run(() -> {
      this.setPosition(leftPos, rightPos);
    })
        .until(this::reachedPosition)
        .beforeStarting(this.enableBrakeMode())
        .beforeStarting(this.disableRampRate())
        .withName("positionDrive");
  }

  /**
   * Returns whether or not the robot has reached the desired position
   */
  private boolean reachedPosition() {
    return this.leftMotorController.reachedSetpoint(this.getLeftPositionInches(), CHASSIS.TOLERANCE) &&
        this.rightMotorController.reachedSetpoint(this.getRightPositionInches(), CHASSIS.TOLERANCE);
  }

  /**
   * Uses PID along with limelight data to turn to target
   */
  public Command balance() {
    return new PIDCommand(
        new PIDController(
            CHASSIS.BALANCE_CONSTANTS.kP,
            CHASSIS.BALANCE_CONSTANTS.kI,
            CHASSIS.BALANCE_CONSTANTS.kD),
        // Close the loop on the turn rate
        this.gyro::getPitch,
        // Setpoint is 0
        0.7,
        // Pipe the output to the turning controls
        (output) -> this
            .driveStraight(
                output > 0 ? (output + CHASSIS.BALANCE_CONSTANTS.kFF) : (output - CHASSIS.BALANCE_CONSTANTS.kFF)),

        // Require the robot drive
        this)
        .andThen(this::emergencyStop)
        .beforeStarting(this::enableBrakeMode)
        .beforeStarting(this::disableRampRate);
  }

  /**
   * Uses PID along with limelight data to turn to target
   */
  public Command getColor() {
    Color detectedColor = m_colorSensor.getColor();

    return run(() -> {
      this.m_colorSensor.getColor();
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.green);

    });
  }

  public Command goToTarget() {
    return new PIDCommand(
        new PIDController(
            CHASSIS.TARGET_CONSTANTS.kP,
            CHASSIS.TARGET_CONSTANTS.kI,
            CHASSIS.TARGET_CONSTANTS.kD),
        // Close the loop on the turn rate
        this.limelight::getTargetY,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        (output) -> this
            .driveStraight(
                -(output > 0 ? (output + CHASSIS.TARGET_CONSTANTS.kFF) : (output - CHASSIS.TARGET_CONSTANTS.kFF))),
        // Require the robot drive
        this)
        // .until(this::reachedTarget)
        .andThen(this::emergencyStop)
        .until(() -> {
          return this.limelight.getTargetY() <= VISION.LIMELIGHT_TOLERANCE
              && this.limelight.getTargetY() >= -VISION.LIMELIGHT_TOLERANCE;
        })
        .beforeStarting(this::enableBrakeMode)
        .beforeStarting(this::disableRampRate)
        .beforeStarting(() -> this.limelight.setLedMode(3))
        .andThen(this::resetLimelight);
  }

  /**
   * Returns whether or not the robot has reached the limelight target
   */
  public boolean reachedTarget() {
    return MISC.WITHIN_TOLERANCE(this.limelight.getTargetY(), 0, VISION.LIMELIGHT_TOLERANCE);
  }

  /**
   * Uses PID along with limelight data to turn to target
   */
  public Command seekTarget() {
    return new PIDCommand(
        new PIDController(
            CHASSIS.SEEK_CONSTANTS.kP,
            CHASSIS.SEEK_CONSTANTS.kI,
            CHASSIS.SEEK_CONSTANTS.kD),
        // Close the loop on the turn rate
        this.limelight::getTargetX,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        (output) -> this
            .turn(output > 0 ? (output + CHASSIS.SEEK_CONSTANTS.kFF) : (output - CHASSIS.SEEK_CONSTANTS.kFF)),
        // Require the robot drive
        this)
        // .until(this::alignedTarget)
        .andThen(() -> this.emergencyStop().schedule())
        .beforeStarting(this.enableBrakeMode())
        .beforeStarting(this.disableRampRate())
        .beforeStarting(() -> this.limelight.setLedMode(3))
        .andThen(this::resetLimelight);
  }

  /**
   * Returns whether or not the robot has aligned with the limelight target
   */
  public boolean alignedTarget() {
    return MISC.WITHIN_TOLERANCE(this.limelight.getTargetX(), 0, VISION.LIMELIGHT_TOLERANCE);
  }

  /**
   * Uses PID along with gyro data to turn to a provided heading
   */
  public Command turnToGyro(double angle) {
    return new PIDCommand(
        new PIDController(
            CHASSIS.ALIGN_CONSTANTS.kP,
            CHASSIS.ALIGN_CONSTANTS.kI,
            CHASSIS.ALIGN_CONSTANTS.kD),
        // Close the loop on the turn rate
        this.gyro::getYaw,
        // Setpoint is 0
        angle,
        // Pipe the output to the turning controls
        (output) -> this
            .turn(output > 0 ? (output + CHASSIS.ALIGN_CONSTANTS.kFF) : (output - CHASSIS.ALIGN_CONSTANTS.kFF)),
        // Require the robot drive
        this)
        .andThen(() -> this.emergencyStop().schedule())
        .beforeStarting(this.enableBrakeMode())
        .beforeStarting(this.disableRampRate());
  }

  public void resetLimelight() {
    this.limelight.setLedMode(1);
  }

  /**
   * Returns a command that enables brake mode on the drivetrain.
   */
  public Command enableBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kBrake));
  }

  /**
   * Returns a command that release brake mode on the drivetrain.
   */
  public Command releaseBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kCoast));
  }

  /**
   * sets the chassis brake mode
   */
  private void setBrakeMode(IdleMode idleMode) {
    this.frontLeftMotor.setIdleMode(idleMode);
    this.frontRightMotor.setIdleMode(idleMode);
    this.pushControllerUpdate();
    SmartDashboard.putBoolean("Brake Mode", idleMode == IdleMode.kBrake);
  }

  /**
   * Returns a command that enables ramp rate on the drivetrain.
   */
  public Command enableRampRate() {
    return runOnce(() -> this.setRampRate(true));
  }

  /**
   * Returns a command that disables ramp rate on the drivetrain.
   */
  public Command disableRampRate() {
    return runOnce(() -> this.setRampRate(false));
  }

  private void setRampRate(boolean state) {
    this.frontLeftMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
    this.frontRightMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
    this.pushControllerUpdate();
    SmartDashboard.putBoolean("Ramping", state);
  }

  /**
   * Returns a command that stops the drivetrain its tracks.
   */
  public Command emergencyStop() {
    return startEnd(() -> {
      this.frontLeftMotor.disable();
      this.frontRightMotor.disable();
    }, this::releaseBrakeMode)
        .beforeStarting(this::enableBrakeMode)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Basically like e-stop command for disabled mode only
   */
  public void killSwitch() {
    this.frontLeftMotor.disable();
    this.frontRightMotor.disable();
    this.setBrakeMode(IdleMode.kBrake);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param forward linear motion [-1 --> 1] (Backwards --> Forwards)
   * @param rot     rotational motion [-1 --> 1] (Left --> Right)
   */
  private void arcadeDrive(double forward, double rot) {
    // Decreasing the drive command for safety
    this.drive.arcadeDrive(forward * 0.5, rot * 0.5);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param percent linear motion [-1 --> 1] (Backwards --> Forwards)
   */
  private void driveStraight(double percent) {
    this.frontLeftMotor.set(percent);
    this.frontRightMotor.set(percent);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param percent linear motion [-1 --> 1] (Backwards --> Forwards)
   */
  private void turn(double percent) {
    this.frontLeftMotor.set(-percent);
    this.frontRightMotor.set(percent);
  }

  /**
   * Sets the drivetrain's maximum percent output
   * 
   * @param maxOutput in percent decimal
   */
  private void setMaxOutput(double maxOutput) {
    this.drive.setMaxOutput(maxOutput);
  }

  /**
   * Sets robot position in inches
   * 
   * @param left  position in inches
   * @param right position in inches
   */
  private void setPosition(double left, double right) {
    this.leftMotorController.setSmartPosition(left);
    this.rightMotorController.setSmartPosition(right);
  }

  /**
   * Returns the drivetrain's left encoder position in inches and meters respectively
   * 
   * @return Left Position
   */
  private double getLeftPositionInches() {
    return this.leftEncoder.getPosition();
  }
  private double getLeftPositionMeters() {
    return this.leftEncoder.getPosition() * 0.0254;
  }

  /**
   * Returns the drivetrain's right encoder position in inches and meters respectively
   * 
   * @return Right Position
   */
  private double getRightPositionInches() {
    return this.rightEncoder.getPosition();
  }
  private double getRightPositionMeters() {
    return this.rightEncoder.getPosition() * 0.0254;
  }
  
  /**
   * Returns the drivetrain's average encoder position in inches
   * 
   * @return Average Position
   */
  public double getAveragePositionInches() {
    return (this.getLeftPositionInches() + this.getRightPositionInches()) / 2;
  }

  /**
   * Returns the drivetrain's left encoder velocityin inches and meters per second respectively
   * 
   * @return Left Velocity
   */
  private double getLeftVelocityInches() {
    return this.leftEncoder.getVelocity();
  }
  private double getLeftVelocityMeters() {
    return this.leftEncoder.getVelocity() * 0.0254;
  }

  /**
   * Returns the drivetrain's right encoder velocity in inches and meters per second respectively
   * 
   * @return Right Velocity
   */
  private double getRightVelocityInches() {
    return this.rightEncoder.getVelocity();
  }
  private double getRightVelocityMeters() {
    return this.rightEncoder.getVelocity() * 0.0254;
  }

  /**
   * Returns the drivetrain's average encoder velocty in inches and meters per second respectively
   * 
   * @return Average Velocity
   */
  public double getAverageVelocityInches() {
    return (this.getLeftVelocityInches() + this.getRightVelocityInches()) / 2;
  }
  public double getAverageVelocityMeters() {
    return (this.getLeftVelocityMeters() + this.getRightVelocityMeters()) / 2;
  }

  /**
   * Resest all drivetrain encoder positions
   */
  public void resetEncoders() {
    this.leftEncoder.setPosition(0);
    this.rightEncoder.setPosition(0);
  }

  /**
   * Updates motor controllers after settings change
   */
  private void pushControllerUpdate() {
    this.frontLeftMotor.burnFlash();
    this.frontRightMotor.burnFlash();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setVoltageOutput(double leftVolts, double rightVolts) {
    this.frontLeftMotor.setVoltage(leftVolts);
    this.frontRightMotor.setVoltage(rightVolts);
    this.drive.feed();
  }

   /**
   * Resest drivetrain gyro position
   */
  public void resetGyro() {
    this.gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in DEGREES, from -180 to 180
   */
  public double getHeadingDeg() {
    return this.gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in DEGREES per second
   */
  public double getTurnRateDeg() {
    return -this.gyro.getRate();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in RADIANS per second
   */
  public double getTurnRateRad() {
    return -this.gyro.getRate() * (Math.PI / 180);
  }

  /**
   * Returns a ChassisSpeeds class with robot data (used for pathplanner).
   *
   * @return some data in the form of ChassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(getAverageVelocityMeters(), 0, getTurnRateRad());
  }

  /* This function drives the robot using arcadeDrive, which would normally use values from -1 to 1.
   * We're instead passing a m/s value, which might be causing some inaccuracies.
   * They might also be PID problems.
   */
  public void setChassisSpeeds(ChassisSpeeds speed) {
    double linearSpeed = speed.vxMetersPerSecond;
    double rotSpeed = speed.omegaRadiansPerSecond;
    //dividing by 3.4 because that's more or less the maximum speed of the robot
    arcadeDrive(linearSpeed / 3.4, rotSpeed);
  }

  /* Gets the position of the robot via pose2d
  */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /* Resets the position of the robot via pose2d
  */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  /* Resets the position of the robot to 0, 0
  */
  public void resetFieldPosition() {
    resetEncoders();
    odometry.resetPosition(new Rotation2d(0, 0), 0, 0, new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.drive.feed();

    odometry.update(
        gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

    printToDashBoard();
    System.out.println(getHeadingDeg());
  }

  /* Prints all values to the dashboard.
  */
  public void printToDashBoard() {
    SmartDashboard.putNumber("Pitch", this.gyro.getPitch());
    SmartDashboard.putNumber("Left Encoder Position", this.getLeftPositionMeters());
    SmartDashboard.putNumber("Right Encoder Position", this.getRightPositionMeters());
  
    SmartDashboard.putNumber("Left Encoder Velocity", this.getLeftVelocityMeters());
    SmartDashboard.putNumber("Right Encoder Velocity", this.getRightVelocityMeters());

    SmartDashboard.putNumber("Forward Velocity", this.getAverageVelocityMeters());
    SmartDashboard.putNumber("Rotational Velocity", this.getTurnRateRad());

    SmartDashboard.putNumber("Heading", this.getHeadingDeg());

    SmartDashboard.putNumber("X Position", this.getPose().getX());
    SmartDashboard.putNumber("Y Position", this.getPose().getY());
  }
  //______________________________________________________________________________________________________

  //SysId

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  // Create a new SysId routine for characterizing the drive.

private SysIdRoutine newSysId(){
   final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                frontLeftMotor.setVoltage(volts.in(Volts));
                frontRightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the front left motor.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-front-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getDistanceTravelled(leftEncoder), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getRelativeRate(leftEncoder), MetersPerSecond));
                // Record a frame for the front right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-front-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getDistanceTravelled(rightEncoder), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getRelativeRate(rightEncoder), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
              
              // Returns sysIdRoutine
              return m_sysIdRoutine;
}

            private double getDistanceTravelled(RelativeEncoder m_encoder){
              double m_wheelCircumference = Math.PI * 6.0;
              double m_distancePerCount = m_wheelCircumference / 8192.0;
              double m_encoderCounts = m_encoder.getPosition(); // Get the current encoder position
              return m_encoderCounts * m_distancePerCount;
            }
              
            private double getRelativeRate(RelativeEncoder m_encoder) {
               double rpm = m_encoder.getVelocity(); // Get the RPM from the encoder
               double distancePerMinute = rpm * Math.PI * 6.0;
               double distancePerSecond = distancePerMinute / 60.0;
               return distancePerSecond;
            }
    
            public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
              SmartDashboard.putBoolean("running Quasustatic: ", true);

              return newSysId().quasistatic(direction);
            }
          
            public Command sysIdDynamic(SysIdRoutine.Direction direction) {
              SmartDashboard.putBoolean("running Dynamic: ", true);
              return newSysId().dynamic(direction);
            }
  
}
