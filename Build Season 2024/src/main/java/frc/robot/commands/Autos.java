// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license fi


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;

//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */

  private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
    Map.entry("Stop", new InstantCommand(RobotContainer.m_drivetrain::stopSwerve))

));

//private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  //  RobotContainer.m_drivetrain::getPose,
    //RobotContainer.m_drivetrain::resetOdometry,
    //Constants.AUTO_TRANSLATION_CONSTANTS,
    //Constants.AUTO_ROTATION_CONSTANTS,
    //RobotContainer.m_drivetrain::setChassisSpeeds,
    //eventMap,
    //true,
    //RobotContainer.m_drivetrain
  //);

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }



  public static Command none() {
    return Commands.none();
  }

}