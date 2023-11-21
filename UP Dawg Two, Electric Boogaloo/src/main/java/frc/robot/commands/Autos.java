// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license fi


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoCommands.AutonArmToHigh;
import frc.robot.commands.AutoCommands.AutonIntake;
import frc.robot.commands.AutoCommands.RunIndexWithBeamBreak;
import frc.robot.commands.AutoCommands.StopIntake;
import frc.robot.commands.ButtonCommands.ArmToIndex;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */

  private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
    Map.entry("Stop", new InstantCommand(RobotContainer.m_drivetrain::stopSwerve)),

    Map.entry("Start", new SequentialCommandGroup(
      new InstantCommand((RobotContainer.m_hand::extend)),
      new AutonArmToHigh(RobotContainer.m_arm),
      new InstantCommand((RobotContainer.m_arm::extend)),
      new WaitCommand(.7),
      new InstantCommand((RobotContainer.m_hand::retract)),
      new WaitCommand(.6),
      new InstantCommand((RobotContainer.m_arm::retract))
      )),

    Map.entry("1stCubeTo2ndCube", new SequentialCommandGroup(
      new ArmToIndex(RobotContainer.m_arm)
      )),

    Map.entry("PickUp2ndCube", new SequentialCommandGroup(
      new InstantCommand((RobotContainer.m_intake::extend)),
      new AutonIntake(RobotContainer.m_topRoller)
      )),

    Map.entry("Got2ndCube", new SequentialCommandGroup(
      new StopIntake(RobotContainer.m_topRoller),
      new InstantCommand((RobotContainer.m_intake::retract)),
      new RunIndexWithBeamBreak(RobotContainer.m_index),
      new InstantCommand((RobotContainer.m_hand::extend))
      )),

    Map.entry("Before2ndCube", new SequentialCommandGroup(
      new AutonArmToHigh(RobotContainer.m_arm)

      )),

    Map.entry("2ndCube", new SequentialCommandGroup(
      new AutonArmToHigh(RobotContainer.m_arm),
      new InstantCommand((RobotContainer.m_hand::retract)),
      new WaitCommand(.2)
    ))

));

private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    RobotContainer.m_drivetrain::getPose,
    RobotContainer.m_drivetrain::resetOdometry,
    Constants.AUTO_TRANSLATION_CONSTANTS,
    Constants.AUTO_ROTATION_CONSTANTS,
    RobotContainer.m_drivetrain::setChassisSpeeds,
    eventMap,
    true,
    RobotContainer.m_drivetrain
  );

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }



  public static CommandBase none() {
    return Commands.none();
  }

  public static CommandBase twoCube() {
    return autoBuilder.fullAuto(PathPlanner.loadPathGroup("2 Cube", new PathConstraints(3, 2)));
  }

}
