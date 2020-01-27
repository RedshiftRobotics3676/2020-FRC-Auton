/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class Autocommand {
  public Command Autodrive;

  public Autocommand(DriveTrain kDriveTrain) {
  var autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      10);

  TrajectoryConfig config =
  new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                      Constants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);

  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  new Pose2d(0, 0, new Rotation2d(0)),
  List.of(
      new Translation2d(1, 1),
      new Translation2d(2, -1)
  ),
  new Pose2d(3, 0, new Rotation2d(0)),
  config
  );

  RamseteCommand ramseteCommand = new RamseteCommand(
  exampleTrajectory,
  kDriveTrain::getPose,
  new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
  new SimpleMotorFeedforward(Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
  Constants.kDriveKinematics,
  kDriveTrain::getWheelSpeeds,
  new PIDController(Constants.kPDriveVel, 0, 0),
  new PIDController(Constants.kPDriveVel, 0, 0),
  kDriveTrain::tankDriveVolts,
  kDriveTrain
  );

  Autodrive = ramseteCommand.andThen(() -> kDriveTrain.tankDriveVolts(0, 0));
  }

  public Command Autodrive() {
    return Autodrive;
  }
}