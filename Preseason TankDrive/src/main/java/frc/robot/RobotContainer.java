// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.StickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.elevator;
import edu.wpi.first.wpilibj.PS4Controller;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final elevator m_Elevator = new elevator();

  private final PS4Controller fullControl = new PS4Controller(Constants.OperatorConstants.kFullControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drivetrain.setDefaultCommand(new StickDrive(m_drivetrain, ()->fullControl.getRawAxis(0), ()->fullControl.getRawAxis(5) ));

  }

  private void configureBindings() {
 
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
