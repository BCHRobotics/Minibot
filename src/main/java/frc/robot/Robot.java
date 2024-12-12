package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  private Timer timer;


 /** This method is called once when the robot starts up. */
@Override
public void robotInit() { 

  robotContainer = new RobotContainer();
  timer = new Timer();

  public class RobotContainer {
    private final Drivetrain m_drivetrain = new Drivetrain();

  }

  public DriveTrain() {
    this.leftMotor = new CANSparkMax(Constants.CHASSIS.LEFT_MOTOR_ID, MotorType.kBrushless);
    this.rightMotor = new CANSparkMax(Constants.CHASSIS.RIGHT_MOTOR_ID, MotorType.kBrushless);
  }
   
}

/** This method is called periodically, regardless of the robot mode. */
@Override
public void robotPeriodic() { 
  CommandScheduler.getinstance().run();
   
}

/** This method is called once when teleop mode (driver control) starts. */
@Override
public void teleopInit() { 
   
}

/** This method is called periodically during teleop mode. */
@Override
public void teleopPeriodic() { 
   
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
   
}

/** This method is called periodically during disabled mode. */
@Override
public void disabledPeriodic() { 
   
}

}
