/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;

  public Auto(Drivetrain drive) 
  {
    drivetrain = drive;
    drivetrain.setRamp(SpeedConstants.autoDriveRampSpeed);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() 
  {
    drivetrain.drive(0, .25);
  }

  @Override
  public void execute() 
  {
    
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.setRamp(SpeedConstants.driveRampSpeed);
    drivetrain.drive(0, 0);
  }

  @Override
  public boolean isFinished() 
  {
    return drivetrain.leftEncoder() > AutoConstants.driveDistance;
  }
}
