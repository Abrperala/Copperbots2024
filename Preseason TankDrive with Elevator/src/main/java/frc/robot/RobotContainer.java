// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SetElevator;
import frc.robot.commands.SetElevatorHome;
import frc.robot.commands.StickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Elevator m_elevator = new Elevator();

  private final PS4Controller fullControl = new PS4Controller(Constants.OperatorConstants.kFullControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drivetrain.setDefaultCommand(new StickDrive(m_drivetrain, ()->fullControl.getRawAxis(0), ()->fullControl.getRawAxis(5) ));

  }

  private void configureBindings() {
 
    new JoystickButton(fullControl, 6).onTrue(new SetElevator(m_elevator));
    new JoystickButton(fullControl, 5).onTrue(new SetElevatorHome(m_elevator));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /**public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  } **/
}
