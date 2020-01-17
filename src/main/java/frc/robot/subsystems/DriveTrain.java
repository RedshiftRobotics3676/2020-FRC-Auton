/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ManualDrive;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  
  WPI_TalonSRX LeftTalon, RightTalon;
  WPI_VictorSPX LeftVictor, RightVictor;
  DifferentialDrive Base;
  SpeedControllerGroup LeftMotors, RightMotors;
  public AHRS ahrs;
  private DifferentialDriveOdometry m_odometry;
  public double LeftSpeed, RightSpeed;
  public final double kDistance = 0.0;
  public final boolean kGyroReversed = false;
  public boolean Manual;

  public DriveTrain() {
    LeftTalon = new WPI_TalonSRX(Constants.LeftTalon);
    RightTalon = new WPI_TalonSRX(Constants.RightTalon);
    LeftVictor = new WPI_VictorSPX(Constants.LeftVictor);
    RightVictor = new WPI_VictorSPX(Constants.RightVictor);
    
    LeftMotors = new SpeedControllerGroup(LeftTalon, LeftVictor);
    RightMotors = new SpeedControllerGroup(RightTalon, RightVictor);
    Base = new DifferentialDrive(LeftMotors, RightMotors);

    ahrs = new AHRS(Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    
    Base.setSafetyEnabled(false);

    LeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    RightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    LeftTalon.setSelectedSensorPosition(0);
    RightTalon.setSelectedSensorPosition(0);
  }

  public void SetDefault(ManualDrive kManualDrive) {
    setDefaultCommand(kManualDrive);
  }

  public void SetSpeed(double Left, double Right) {
    LeftSpeed = -Left;
    RightSpeed = -Right;
  }

  @Override
  public void periodic() {
    if (Manual) {
      Base.tankDrive(LeftSpeed * 2, RightSpeed * 2);
    }

    m_odometry.update(Rotation2d.fromDegrees(getHeading()), LeftTalon.getSelectedSensorPosition()*kDistance, RightTalon.getSelectedSensorPosition()*kDistance);
    SmartDashboard.putNumber("Gyro", getHeading());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LeftTalon.getSelectedSensorVelocity()*kDistance, RightTalon.getSelectedSensorVelocity()*kDistance);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    LeftTalon.setSelectedSensorPosition(0);
    RightTalon.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (LeftTalon.getSelectedSensorPosition()*kDistance + RightTalon.getSelectedSensorPosition()*kDistance) / 2.0;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    Base.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    ahrs.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }
}
