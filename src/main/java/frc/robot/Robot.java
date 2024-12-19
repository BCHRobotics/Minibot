package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private Drivetrain m_drivetrain;
  private Timer timer;

  public Drivetrain getDriveTrain(){
    return m_drivetrain;
  }


 /** This method is called once when the robot starts up. */
@Override
public void robotInit() {
  CommandScheduler.getInstance().run(); 

  robotContainer = new RobotContainer();
  timer = new Timer(); //WPILib timer
   
}

/** This method is called periodically, regardless of the robot mode. */
@Override
public void robotPeriodic() {
  // this is calling the methods from drive train method names motorTemperature and motorVoltage
  robotContainer.getDriveTrain().motorTemperature();
  robotContainer.getDriveTrain().motorVoltage();
   
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

  robotContainer.getDriveTrain().brake();
  //reset then start timer
  timer.reset();
  timer.start();
}
/** This method is called periodically during disabled mode. */
@Override
public void disabledPeriodic() { 
  //check if 3 second have elapsed
  if(timer.hasElapsed(3.0)){
    //switch to coast mode
    robotContainer.getDriveTrain().releaseBrakes();
    timer.stop();
  }
   
}

}
