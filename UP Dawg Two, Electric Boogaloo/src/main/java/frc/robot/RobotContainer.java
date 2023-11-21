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
import frc.robot.commands.ButtonCommands.IndexReverse;
import frc.robot.commands.ButtonCommands.RunIndex;
import frc.robot.commands.ButtonCommands.TopIntakeGoBrrrrrrr;
import frc.robot.commands.ButtonCommands.TopIntakeGoBrrrrrrrBackwards;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomRoller;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  public static final TopRoller m_topRoller = new TopRoller();
  public static final BottomRoller m_bottomRoller = new BottomRoller();
  public static final Hand m_hand = new Hand();
  public static final Index m_index = new Index();
  private static final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();


  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  private final PS4Controller operator = new PS4Controller(1);

  /* Drive Controls */
  private final int translationAxis = PS4Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS4Controller.Axis.kRightX.value;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_autoChooser.setDefaultOption("None", Autos.none());
    m_autoChooser.addOption("2Cube", Autos.twoCube());
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

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {


    // sets the operator controller options button to reset the drive gyro
    new JoystickButton(operator, 10).onTrue(new InstantCommand(m_drivetrain::zeroGyro));

        // sets the operator controller square button to the command ArmToInDex
    new JoystickButton(operator, 1).onTrue(new SequentialCommandGroup(new InstantCommand(m_arm::retract), new WaitCommand(1), new ArmToIndex(m_arm)));
   
    // sets the operator controller X button to the command ArmToSecondNode
    new JoystickButton(operator, 2).onTrue(new ArmToSecondNode(m_arm));

    // sets the operator circle square button to the command ArmToThirdNode
    new JoystickButton(operator, 3).onTrue(new ArmToThirdNode(m_arm));


    new JoystickButton(operator, 4).onTrue(new ArmToPlayerStation(m_arm));
    
    // sets the first left bumper to the command HandClose
    new JoystickButton(operator, 5).onTrue(new InstantCommand(m_hand::retract));

    // sets the first left bumper to the command HandOpen 
    new JoystickButton(operator, 6).onTrue(new InstantCommand((m_hand::extend)));

    // sets the operator controller second left bumper to the command TopIntakeGoBrrrrrrr and BottomINtakeGoBrrrrrrr
    new JoystickButton(operator, 7).whileTrue(new ParallelCommandGroup(new TopIntakeGoBrrrrrrrBackwards(m_topRoller), new BottomIntakeGoBrrrrrrrBackwards(m_bottomRoller)));

    // sets the operator controller second right bumper to the command TopIntakeGoBrrrrrrr
    new JoystickButton(operator, 8).whileTrue(new ParallelCommandGroup(new TopIntakeGoBrrrrrrr(m_topRoller), new BottomIntakeGoBrrrrrrr(m_bottomRoller)));

    // sets the operator controller Share button to the RunIndex Command
    new JoystickButton(operator, 9).whileTrue(new IndexReverse(m_index));

    // sets the operator controller options button to the IndexReverse Command
    //new JoystickButton(operator, 10).whileTrue(new RunIndex(m_index));

    // sets the operator controller ps4 button  to the command togglePiston on intake
    new JoystickButton(operator, 13).onTrue(new InstantCommand(m_intake::togglePiston));

     //sets the operator controller center pad to the command togglePiston on arm
    new JoystickButton(operator, 14).onTrue(new InstantCommand(m_arm::togglePiston));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
