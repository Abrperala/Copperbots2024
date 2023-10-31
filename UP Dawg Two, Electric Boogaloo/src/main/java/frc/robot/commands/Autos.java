// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license fi


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return null;
  }

  /*private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    RobotContainer.m_drivetrain::getPose,
    RobotContainer.m_drivetrain::resetOdometry,
    AUTO_TRANSLATION_CONSTANTS,
    AUTO_ROTATION_CONSTANTS, 
    RobotContainer.m_drivetrain::setChassisSpeeds,
    eventMap,
    true,
    RobotContainer.m_drivetrain
  ); */

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
