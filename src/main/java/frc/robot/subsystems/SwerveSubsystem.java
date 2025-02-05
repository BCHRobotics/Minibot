package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.SwerveDrive;
import swervelib.swerveDrive;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity


public class SwerveSubsystem extends SubsystemBase {

  double maximumSpeed = Units.feetToMeters(4.5);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
// Initialize swerve drive
  SwerveDrive swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);


  SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
      return run(() -> {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
        true,
        false);
        Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                        headingX.getAsDouble(),
                                                                        headingY.getAsDouble(),
                                                                        swerveDrive.getOdometryHeading().getRadians(),
                                                                        swerveDrive.getMaximumVelocity()));
    });
  }
}
