// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CHASSIS;
import frc.robot.util.control.SparkMaxPID;
import frc.robot.util.devices.Gyro;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax frontLeftMotor;
  private final CANSparkMax frontRightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DifferentialDrive drive;
  private final Gyro gyro;
  private final SparkMaxPID leftPIDMotorController;
  private final SparkMaxPID rightPIDMotorController;
  private final DifferentialDriveOdometry odometry;
  private final DifferentialDrive differentialDrive;
  public static final double LEFT_DRIVE_CONSTANTS;
  public static final double RIGHT_DRIVE_CONSTANTS;

  
    /** Creates a new Drive subsystem. */
    public Drivetrain() {
      // Init the motors
      this.frontLeftMotor = new CANSparkMax(CHASSIS.FRONT_LEFT_ID, MotorType.kBrushless);
      this.frontRightMotor = new CANSparkMax(CHASSIS.FRONT_RIGHT_ID, MotorType.kBrushless);
      
      this.frontLeftMotor.restoreFactoryDefaults();
      this.frontRightMotor.restoreFactoryDefaults();
      // The motors are being set to coast mode here, but they are set to brake in the drive commands
      this.frontLeftMotor.setIdleMode(IdleMode.kCoast);
      this.frontRightMotor.setIdleMode(IdleMode.kCoast);
  
      this.frontLeftMotor.setSmartCurrentLimit(60, 20);
      this.frontRightMotor.setSmartCurrentLimit(60, 20);
  
      this.frontLeftMotor.setInverted(CHASSIS.INVERTED);
      this.frontRightMotor.setInverted(!CHASSIS.INVERTED);
  
      this.leftEncoder = this.frontLeftMotor.getEncoder();
      this.rightEncoder = this.frontRightMotor.getEncoder();

      this.leftEncoder.setPositionConversionFactor(CHASSIS.LEFT_POSITION_CONVERSION());
      this.rightEncoder.setPositionConversionFactor(CHASSIS.RIGHT_POSITION_CONVERSION());

      this.leftEncoder.setVelocityConversionFactor(CHASSIS.LEFT_VELOCITY_CONVERSION());
      this.rightEncoder.setVelocityConversionFactor(CHASSIS.RIGHT_VELOCITY_CONVERSION());

      this.leftPIDMotorController = new SparkMaxPID(this.frontLeftMotor, CHASSIS.LEFT_DRIVE_CONSTANTS);
      this.rightPIDMotorController = new SparkMaxPID(this.frontRightMotor, CHASSIS.RIGHT_DRIVE_CONSTANTS);

      this.leftPIDMotorController.setFeedbackDevice(this.leftEncoder);
      this.rightPIDMotorController.setFeedbackDevice(this.rightEncoder);

      this.gyro = Gyro.getInstance();

    this.leftPIDMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);
    this.rightPIDMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);

      this.differentialDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor); 

      this.odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

    this.drive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);

    AutoBuilder.configureLTV(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            0.02, // PPLTVController is the built in path following controller for differential drive trains
            new ReplanningConfig(), // The robot configuration
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


  // START ADDING YOUR CODE HERE
  // Add arcadeDriveCommand, setbrakemode, etc.
  // Day 9 slideshow
  // ignore the other methods this

  public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier turn) {
    return run(() -> this.differentialDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble()));

  }





  
  // ignore belo 
  public void setDeadband(double deadband) {
    this.drive.setDeadband(deadband);
  }

  /**
   * Command to turn the robot to a heading
   */
  public Command alignToHeadingCommand(DoubleSupplier current, DoubleSupplier desired) {
    return runOnce(() -> {
      this.arcadeDrive(0, Math.min((Rotation2d.fromDegrees(desired.getAsDouble()).minus(Rotation2d.fromDegrees(current.getAsDouble())).getDegrees()) * 0.02, 0.1));
    })
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("headingAlign");
  }

  public double getDesiredHeading() {
    return 90;
  }

  
  /**
   * Sets motor output using arcade drive controls
   * 
   * @param forward linear motion [-1 --> 1] (Backwards --> Forwards)
   * @param rot     rotational motion [-1 --> 1] (Left --> Right)
   */
  public void arcadeDrive(double forward, double rot) {
    // Decreasing the drive command for safety
    this.drive.arcadeDrive(forward, rot);
  }

  /**
   * Sets the drivetrain's maximum percent output
   * 
   * @param maxOutput in percent decimal
   */
  public void setMaxOutput(double maxOutput) {
    this.drive.setMaxOutput(maxOutput);
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
    arcadeDrive(Math.min(linearSpeed / 3.4, 0.5), Math.min(rotSpeed, 0.5));
  }

  /* Gets the position of the robot via pose2d
  */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /* Gets the position of the robot via pose2d
  */
  public Pose2d getDesiredPose() {
    return new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
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

    // update odometry below
    
    // This method will be called once per scheduler run
    this.drive.feed();
    
    printToDashBoard();
    //System.out.println(getHeadingDeg());
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
}