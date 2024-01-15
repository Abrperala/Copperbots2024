package frc.robot;

import frc.robot.subsystems.SwerveDrivetrain;

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
    // runs method below to configure button bindings
    configureBindings();
    configureAutos();

    // add auton options in the constructor as it only happens once
    // m_autoChooser.setDefaultOption("None", Autos.none());
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
    SmartDashboard.putData("Chosen Auton?", m_autoChooser.getSelected());
    // this may do what I intended the above to do
    SmartDashboard.putString("Actually chosen Auton?", m_autoChooser.toString());
  }
}