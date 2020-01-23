/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Vision extends SequentialCommandGroup {
  /**
   * Creates a new Vision.
   */
  /*public static final double ksVolts = 0.919;
  public static final double kvVoltSecondsPerMeter = 0.0633;
  public static final double kaVoltSecondsSquaredPerMeter = 0.00194;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 0.0688;
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  public static final double kTrackwidthMeters = 0.56;
  public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  
  public Vision(DriveTrain kDriveTrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new VisionAiming(kDriveTrain), Ramsette(kDriveTrain));
  }
  
  public static Command Ramsette(DriveTrain kDriveTrain) {
    final Trajectory exampleTrajectory = Constants.t;
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        kDriveTrain::getPose,
        new RamseteController(kRamseteB, kRamseteZeta),
        new SimpleMotorFeedforward(ksVolts,
                                  kvVoltSecondsPerMeter,
                                  kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        kDriveTrain::getWheelSpeeds,
        new PIDController(kPDriveVel, 0, 0),
        new PIDController(kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        kDriveTrain::tankDriveVolts,
        kDriveTrain
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> kDriveTrain.tankDriveVolts(0, 0));
  }

  public static Command Ramsette(Trajectory t, DriveTrain kDriveTrain) {
    RamseteCommand ramseteCommand = new RamseteCommand(
      t,
      kDriveTrain::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(ksVolts,
                                kvVoltSecondsPerMeter,
                                kaVoltSecondsSquaredPerMeter),
      kDriveKinematics,
      kDriveTrain::getWheelSpeeds,
      new PIDController(kPDriveVel, 0, 0),
      new PIDController(kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      kDriveTrain::tankDriveVolts,
      kDriveTrain
  );

  // Run path following command, then stop at the end.
  return ramseteCommand.andThen(() -> kDriveTrain.tankDriveVolts(0, 0));
  }
}*/