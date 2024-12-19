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
  timer = new Timer(); //WPILib Timer
}

/** This method is called periodically, regardless of the robot mode. */
@Override
public void robotPeriodic() { 
   CommandScheduler.getInstance().run();
   
  this.robotContainer.getDrivetrain().getTemperature();
  this.robotContainer.getDrivetrain().getBusVoltage();
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
   
robotContainer.getDrivetrain().brake();

//Reset then Start the timer
timer.reset();
timer.start();

}


/** This method is called periodically during disabled mode. */
@Override
public void disabledPeriodic() { 
   
      //Check if 3 seconds have elapsed
  if (timer.hasElapsed(3.0)) {
    //Switch to coast mode
    robotContainer.getDrivetrain().releaseBrakes();
    timer.stop(); //Stop the timer after switching to coast mode
  }
}

}