package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
	public static final int LeftTalon = 6;
	public static final int RightTalon = 1;
	public static final int LeftVictor = 10;
	public static final int RightVictor = 4;

    public static final double ksVolts = 0.14;
    public static final double kvVoltSecondsPerMeter = 1.93;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0444;
	public static final double kPDriveVel = 4.47;
	public static final double kTrackwidthMeters = 0.267;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;
	public static final double DistancePerPulse = 0;
}