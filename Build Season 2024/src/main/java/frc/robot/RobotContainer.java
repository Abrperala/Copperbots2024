package frc.robot;

import frc.robot.commands.Autos;

import frc.robot.subsystems.SwerveDrivetrain;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
  private static final SendableChooser<Command> m_autoChooser = new SendableChooser<>();


  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0); 
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

    // I wanted it to confirm the selected auto, it didnt, TODO: fix it so it displays selected auto
    SmartDashboard.putData("Auto mode", m_autoChooser);
    SmartDashboard.putData("Chosen Auto",  m_autoChooser.getSelected());


    m_drivetrain.setDefaultCommand(
      new SwerveDrive(
          m_drivetrain, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis), 
          () -> false
      )
  );
  }

  private void configureBindings() {
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