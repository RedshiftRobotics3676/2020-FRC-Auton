package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  WPI_VictorSPX LeftVictor = new WPI_VictorSPX(Constants.LeftVictor);
  WPI_TalonSRX LeftTalon = new WPI_TalonSRX(Constants.LeftTalon);
  WPI_VictorSPX RightVictor = new WPI_VictorSPX(Constants.RightVictor);
  WPI_TalonSRX RightTalon = new WPI_TalonSRX(Constants.RightTalon);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(LeftTalon, LeftVictor);

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(RightTalon, RightVictor);

  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  private final Gyro gyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry odometry;

  public DriveTrain() {
    LeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    RightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    resetEncoders();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    double LeftSpeed = RobotContainer.XboxController.getY(Hand.kLeft);
    double RightSpeed = RobotContainer.XboxController.getY(Hand.kRight);
    drive.arcadeDrive(LeftSpeed, RightSpeed);

    odometry.update(Rotation2d.fromDegrees(getHeading()), LeftTalon.getSelectedSensorPosition()*Constants.DistancePerPulse,
                    RightTalon.getSelectedSensorPosition()*Constants.DistancePerPulse);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LeftTalon.getSelectedSensorVelocity()*Constants.DistancePerPulse,LeftTalon.getSelectedSensorVelocity()*Constants.DistancePerPulse);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
  }

  public void resetEncoders() {
    LeftTalon.setSelectedSensorPosition(0);
    LeftTalon.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (LeftTalon.getSelectedSensorPosition() + RightTalon.getSelectedSensorPosition()) * Constants.DistancePerPulse / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public double getTurnRate() {
    return gyro.getRate();
  }
}