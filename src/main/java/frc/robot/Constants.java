package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
	public static final int LeftTalon = 6;
	public static final int RightTalon = 1;
	public static final int LeftVictor = 10;
	public static final int RightVictor = 4;

    public static final double ksVolts = 1.45;
    public static final double kvVoltSecondsPerMeter = 0.129;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0032;
	public static final double kPDriveVel = 1.07/1.27;
	public static final double kTrackwidthMeters = 0.267;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 9.602;
	public static final double kMaxAccelerationMetersPerSecondSquared = 9.602;
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;
	public static final double DistancePerPulse = 1/4096;//0.000116889335944;
}