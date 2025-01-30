package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends TimedRobot {


  private final int LEFT_MOTOR_ID = 13;
  private final int RIGHT_MOTOR_ID = 11; 

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor; 

    private DifferentialDrive robotDrive; 

    private XboxController driverController; 

    private Timer timer;



 


 /** This method is called once when the robot starts up. */
@Override
public void robotInit() { 

  timer = new Timer();

  // Initialzir motors
  this.leftMotor = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
  this.rightMotor = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);
  this.robotDrive = new DifferentialDrive(this.leftMotor, this.rightMotor);
  
  this.driverController = new XboxController(0);


  this.robotDrive.setMaxOutput(0.25); // Limit max speed for safety
  this.robotDrive.setSafetyEnabled(true); // Enable motor safety on the drive object
  this.robotDrive.setExpiration(0.1); // Reset motor controllers to factory defaults
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
}

/** This method is called periodically, regardless of the robot mode. */
@Override
public void robotPeriodic() { 
   


}

/** This method is called once when teleop mode (driver control) starts. */
@Override
public void teleopInit() { 
   
}

/** This method is called periodically during teleop mode. */
@Override
public void teleopPeriodic() { 
  robotDrive.arcadeDrive(-driverController.getLeftY(), -driverController.getRightX());



  if (driverController.getLeftBumper()) {
    this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  } else {
    this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

}


/** This method is called once when autonomous mode starts. */
@Override
public void autonomousInit() {

}

/** This function is called periodically during autonomous. */
@Override
  public void autonomousPeriodic() {

}

/** This method is called once each time the robot enters Disabled mode. */
@Override
public void disabledInit() { 
   this.leftMotor.setIdleMode(IdleMode.kBrake);
   this.rightMotor.setIdleMode(IdleMode.kBrake);


   timer.reset();
   timer.start();
}

/** This method is called periodically during disabled mode. */
@Override
public void disabledPeriodic() { 
   if (timer.hasElapsed(3.0)) {
    this.leftMotor.setIdleMode(IdleMode.kCoast);
    this.rightMotor.setIdleMode(IdleMode.kCoast);
    timer.stop();
   }
}

}
