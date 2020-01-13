/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

WPI_TalonSRX LeftTalon, RightTalon;
WPI_VictorSPX LeftVictor, RightVictor;
DifferentialDrive Base;

  public DriveTrain() {
LeftTalon = new WPI_TalonSRX(Constants.LeftTalon);
RightTalon = new WPI_TalonSRX(Constants.RightTalon);
LeftVictor = new WPI_VictorSPX(Constants.LeftVictor);
RightVictor = new WPI_VictorSPX(Constants.RightVictor);

Base = new DifferentialDrive(LeftTalon, RightTalon);

RightVictor.follow(RightTalon);
LeftVictor.follow(LeftTalon);

    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftTalon.setSelectedSensorPosition(0);
    rightTalon.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
