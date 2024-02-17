package frc.robot;

import frc.robot.subsystems.BasePivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TopPivot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.CopperBotUtils;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SwerveDrivetrain m_drivetrain;
  private static SendableChooser<Command> m_autoChooser;
  public static Shooter m_shooter;
  public static Limelight m_limelight;
  public static TopPivot m_topPivot;
  public static BasePivot m_basePivot;
  public static Intake m_intake;

  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  private final PS4Controller operator = new PS4Controller(1);

  /* Drive Controls */
  private final int translationAxis = PS4Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS4Controller.Axis.kRightX.value;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain = new SwerveDrivetrain();
    m_autoChooser = new SendableChooser<>();
    m_shooter = new Shooter();
    m_limelight = new Limelight();
    m_basePivot = new BasePivot();
    m_topPivot = new TopPivot();
    m_intake = new Intake();

    // runs method below to configure button bindings
    configureBindings();
    configureAutos();

    // add auton options in the constructor as it only happens once
    // m_autoChooser.addOption("none", Autos.none());

    m_drivetrain.setDefaultCommand(
        new SwerveDrive(
            m_drivetrain,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> false));
  }

  private void configureBindings() {

    // new JoystickButton(operator, 1).onTrue(new Shoot(m_shooter));

    new JoystickButton(driver, 2).onTrue(new SequentialCommandGroup(new ResetPoseFromLL(m_limelight, m_drivetrain),
        m_drivetrain.followPathCommand(m_limelight.getTargetPoseFromAlliance())));

    new JoystickButton(driver, 10).onTrue(new InstantCommand(m_drivetrain::zeroGyro));

    new JoystickButton(operator, 7).whileTrue(new Shoot(m_shooter, -.3));

    new JoystickButton(operator, 8).whileTrue(new Intaking(m_intake));

    new JoystickButton(operator, 3).onTrue(new SetTopPivotToAngle(m_topPivot, 0));

    new JoystickButton(operator, 4).onTrue(new SetTopPivotToAngle(m_topPivot, 70));

    // new JoystickButton(operator, 1).onTrue(new InstantCommand(() ->
    // m_basePivot.setPivot(.11)));

    new JoystickButton(operator, 1).onTrue(new SetBasePivotToAngle(m_basePivot,
        0));

    new JoystickButton(operator, 2).onTrue(new SetBasePivotToAngle(m_basePivot,
        90));

  }

  private void configureAutos() {
    NamedCommands.registerCommand("stopDrive", new InstantCommand(m_drivetrain::stopSwerve));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous, chosen by smart dashboard
   */
  public Command getAutonomousCommand() {

    return m_autoChooser.getSelected();
  }

  public void displayAutoChooser() {

    // allows you to choose the options for Auton
    SmartDashboard.putData("Auto mode", m_autoChooser);
    // should confirm your selection for Auton, Im pretty sure it will just show me
    // a button like last time instead of the name of the Auton
    // SmartDashboard.putString("Chosen Auton?",
    // m_autoChooser.getSelected().toString());

  }

  public void addAutonOptions() {
    m_autoChooser.addOption("none", new Command() {
    });
    m_autoChooser.addOption("4pice center", AutoBuilder.buildAuto("4 Piece Center"));

  }
}