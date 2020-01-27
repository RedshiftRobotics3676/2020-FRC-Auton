package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autodrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  public final DriveTrain kDriveTrain = new DriveTrain();
  public static XboxController XboxController = new XboxController(0);
  JoystickButton Y = new JoystickButton(XboxController, 4);
  
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    Y.whileHeld(new Autodrive(kDriveTrain));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}