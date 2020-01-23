/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------
//orentation, and final position
package frc.robot.commands;

import java.util.List;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class VisionAiming extends CommandBase {

   * Creates a new VisionAiming.
   */

   /*DriveTrain kDriveTrain;
   final double testRatio = 2.0;
   final double testDistance = 0.5;
   double ratio, length, distance, angle;
   int step;
   boolean finished;
   public static final double ksVolts = 0.919;
   public static final double kvVoltSecondsPerMeter = 0.0633;
   public static final double kaVoltSecondsSquaredPerMeter = 0.00194;
 
   // Example value only - as above, this must be tuned for your drive!
   public static final double kPDriveVel = 0.0688;
   public static final double kMaxSpeedMetersPerSecond = 3;
   public static final double kMaxAccelerationMetersPerSecondSquared = 3;
   public static final double kTrackwidthMeters = 0.56;
   public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public Trajectory t;

  public VisionAiming(DriveTrain kDriveTrain) {
    addRequirements(kDriveTrain);
    this.kDriveTrain = kDriveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setDouble(1);
    kDriveTrain.Manual = false;
    step = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  //reverse if length is increasing or ratio is changing
  @Override
  public void execute() {
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
      switch (step) {
        case 1:
          StepOne();
          break;
        case 2:
          ratio = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0)/NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
          length = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
          step++;
          break;
        case 3:
          angle = Math.acos(ratio/testRatio);
          distance = testDistance*320/(length*Math.tan(24.85));
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setDouble(0);
          step++;
          break;
        case 4:
          StepFour();
          break;
        default:
          finished = true;
          break;
      }
    }
  }

  void StepOne() {
    double pos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    if (pos > 0.5) {
      kDriveTrain.Base.arcadeDrive(0, -pos-0.1);
    }
    else if (pos < -0.5) {
      kDriveTrain.Base.arcadeDrive(0, -pos+0.1);
    }
    else {
      step = 2;
    }
  }

  void StepFour()  {
    final var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics, 10);
      
    final TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
    kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    final Trajectory Trajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(Math.sin(angle), distance - Math.cos(angle), new Rotation2d(-angle)),
    // Pass config
    config);
    Constants.t = Trajectory;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}*/
