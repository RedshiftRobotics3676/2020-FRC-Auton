/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class ManualDrive extends CommandBase {
  DriveTrain kDriveTrain;
  double LeftSpeed, RightSpeed;
  final double MaxAcceleration = 0.05;
  /**
   * Creates a new ManualDrive.
   */
  public ManualDrive(DriveTrain kDriveTrain) {
    addRequirements(kDriveTrain);
    this.kDriveTrain = kDriveTrain;
    kDriveTrain.SetDefault(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kDriveTrain.Manual = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LeftSpeed = RobotContainer.XboxController.getY(Hand.kLeft);
    RightSpeed = RobotContainer.XboxController.getY(Hand.kRight);
    
    /*if (LeftSpeed-kDriveTrain.LeftSpeed > MaxAcceleration) {
      LeftSpeed = kDriveTrain.LeftSpeed + MaxAcceleration;
    }
    else if (kDriveTrain.LeftSpeed-LeftSpeed > MaxAcceleration) {
      LeftSpeed = kDriveTrain.LeftSpeed - MaxAcceleration;
    }

    if (RightSpeed-kDriveTrain.RightSpeed > MaxAcceleration) {
      RightSpeed = kDriveTrain.RightSpeed + MaxAcceleration;
    }
    else if (kDriveTrain.RightSpeed-RightSpeed > MaxAcceleration) {
      RightSpeed = kDriveTrain.RightSpeed - MaxAcceleration;
    }*/

    kDriveTrain.SetSpeed(LeftSpeed, RightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kDriveTrain.Manual = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
