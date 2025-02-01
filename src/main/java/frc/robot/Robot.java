package frc.robot;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants;



public class Robot extends TimedRobot {
  private RobotContainer robotContainer;


 /** This method is called once when the robot starts up. */
@Override
public void robotInit() { 
  robotContainer = new RobotContainer();
   
}

/** This method is called periodically, regardless of the robot mode. */
@Override
public void robotPeriodic() { 
  CommandScheduler.getInstance().run();

  robotContainer.getElevator().run();

  
   
}

/** This method is called once when teleop mode (driver control) starts. */
@Override
public void teleopInit() { 
   
}

/** This method is called periodically during teleop mode. */
@Override
public void teleopPeriodic() { 
  //robotContainer.getElevator();

  
     
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