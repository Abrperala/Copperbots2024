// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ButtonCommands.ArmToIndex;
import frc.robot.commands.ButtonCommands.ArmToPlayerStation;
import frc.robot.commands.ButtonCommands.ArmToSecondNode;
import frc.robot.commands.ButtonCommands.ArmToThirdNode;
import frc.robot.commands.ButtonCommands.BottomIntakeGoBrrrrrrr;
import frc.robot.commands.ButtonCommands.BottomIntakeGoBrrrrrrrBackwards;
import frc.robot.commands.ButtonCommands.TopIntakeGoBrrrrrrr;
import frc.robot.commands.ButtonCommands.TopIntakeGoBrrrrrrrBackwards;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomRoller;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TopRoller;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
  public static final Arm m_arm = new Arm();
  public static final Intake m_intake = new Intake();
  // why didn't I combine into one? TODO: find out why each roller has its own subsytem and potentially merge into intake subsystem
  public static final TopRoller m_topRoller = new TopRoller();
  public static final BottomRoller m_bottomRoller = new BottomRoller();
  public static final Hand m_hand = new Hand();
  public static final Index m_index = new Index();
  private static final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private static final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();


  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0); // TODO: change buttons to correct controllers
  private final PS4Controller operator = new PS4Controller(1);

  /* Drive Controls */
  private final int translationAxis = PS4Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS4Controller.Axis.kRightX.value;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // runs method below to configure button bindings
    configureBindings();

    m_autoChooser.setDefaultOption("None", Autos.none());
    m_autoChooser.addOption("2Cube", Autos.twoCube());
    // I wanted it to confirm the selected auto, it didnt, TODO: fix it so it displays selected auto
    SmartDashboard.putData("Auto mode", m_autoChooser);
    SmartDashboard.putData("Chosen Auto",  m_autoChooser.getSelected());


    m_drivetrain.setDefaultCommand(
      new SwerveDrive(
          m_drivetrain, 
          () -> -operator.getRawAxis(translationAxis), 
          () -> -operator.getRawAxis(strafeAxis), 
          () -> -operator.getRawAxis(rotationAxis), 
          () -> false
      )
  );
  }

  private void configureBindings() {
    // sets the operator controller square button to move the arm inside the index
    new JoystickButton(operator, 1).onTrue(new SequentialCommandGroup(new InstantCommand(m_arm::retract), new WaitCommand(1), new ArmToIndex(m_arm)));
   
    // sets the operator controller X button to the move the arm to the second node, 93 degrees
    new JoystickButton(operator, 2).onTrue(new ArmToSecondNode(m_arm));

    // sets the operator circle square button to the arm to the third node, 100 degrees
    new JoystickButton(operator, 3).onTrue(new ArmToThirdNode(m_arm));

    //sets the operator triangle botton to the arm to the player station, 87 degrees
    new JoystickButton(operator, 4).onTrue(new ArmToPlayerStation(m_arm));
    
    // sets the first left bumper to open the hand
    new JoystickButton(operator, 5).onTrue(new InstantCommand(m_hand::retract));

    // sets the first right bumper to close the hand
    new JoystickButton(operator, 6).onTrue(new InstantCommand((m_hand::extend)));

    // sets the operator controller left trigger to spin intake backwards (eject)
    new JoystickButton(operator, 7).whileTrue(new ParallelCommandGroup(new TopIntakeGoBrrrrrrrBackwards(m_topRoller), new BottomIntakeGoBrrrrrrrBackwards(m_bottomRoller)));

    // sets the operator controller right trigger to spin the intake forwards (intake)
    new JoystickButton(operator, 8).whileTrue(new ParallelCommandGroup(new TopIntakeGoBrrrrrrr(m_topRoller), new BottomIntakeGoBrrrrrrr(m_bottomRoller)));

    // sets the operator controller Share button to roll the index forward (towards arm)
    //new JoystickButton(operator, 9).whileTrue(new Runindex(m_index));

    // sets the operator controller options button allign with target from limelight
    new JoystickButton(operator, 9).onTrue(new AlignWithTarget(m_limelight, m_drivetrain));

    // sets the operator controller options button to reset the drive gyro
    new JoystickButton(operator, 10).onTrue(new InstantCommand(m_drivetrain::zeroGyro));

    // sets the operator controller ps4 button to toggle the intake deployment
    new JoystickButton(operator, 13).onTrue(new InstantCommand(m_intake::togglePiston));

    //sets the operator controller center pad to toggle the arm extension
    new JoystickButton(operator, 14).onTrue(new InstantCommand(m_arm::togglePiston));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous, chosen by smart dashboard
   */
  public Command getAutonomousCommand() {
    
    return m_autoChooser.getSelected();
  }
}
