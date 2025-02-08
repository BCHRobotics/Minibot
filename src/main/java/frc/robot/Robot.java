package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command autonomousCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // Cancel any autonomous commands
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // Schedule the autonomous command if defined
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Any additional teleop logic (if necessary)
  }

  @Override
  public void autonomousPeriodic() {
    // Reserved for custom autonomous logic
  }

  @Override
  public void disabledInit() {
    // Any cleanup when robot is disabled
  }

  @Override
  public void disabledPeriodic() {
    // Reserved for diagnostics during disabled mode
  }
}
