package frc.robot;

import frc.robot.subsystems.BasePivot;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TopPivot;

import java.util.Map;

import com.ctre.phoenix.time.StopWatch;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.GeneralUtils;
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

	private final Command m_selectCommand;

	// The robot's subsystems and commands are defined here...
	public static SwerveDrivetrain m_drivetrain;
	private static SendableChooser<Command> m_autoChooser;
	public static Shooter m_shooter;
	public static Limelight m_limelight;
	public static TopPivot m_topPivot;
	public static BasePivot m_basePivot;
	public static Intake m_intake;
	public static Candle m_candle;
	public static Climb m_climb;

	/* Controllers */
	private final PS4Controller driver = new PS4Controller(0);
	private final PS4Controller operator = new PS4Controller(1);
	private final PS4Controller testing = new PS4Controller(2);

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
		m_candle = new Candle();
		m_climb = new Climb();

		// An example selectcommand. Will select from the three commands based on the
		// value returned
		// by the selector method at runtime. Note that selectcommand works on Object(),
		// so
		// selector does not have to be an enum; it could be any desired type (string,
		// integer,
		// boolean, double...)
		m_selectCommand = new SelectCommand<>(
				// Maps selector values to commands
				Map.ofEntries(
						Map.entry(CommandSelector.ZERO, new WaitCommand(.1)),
						Map.entry(CommandSelector.ONE,
								new ParallelCommandGroup(
										m_drivetrain.followPathCommandtoAT1(),
										new SequentialCommandGroup(
												new SetTopPivotToAngle(m_topPivot, 87),
												new SetBasePivotToAngle(m_basePivot,
														127),
												new Intaking(m_intake),
												new SetBasePivotToAngle(m_basePivot,
														90),
												new SetTopPivotToAngle(m_topPivot, 0)))),
						Map.entry(CommandSelector.TWO,
								new ParallelCommandGroup(
										m_drivetrain.followPathCommandtoAT2(),
										new SequentialCommandGroup(
												new SetTopPivotToAngle(m_topPivot, 87),
												new SetBasePivotToAngle(m_basePivot,
														127),
												new Intaking(m_intake),
												new SetBasePivotToAngle(m_basePivot,
														90),
												new SetTopPivotToAngle(m_topPivot, 0)))),
						Map.entry(CommandSelector.SEVEN,
								new ParallelRaceGroup(
										new AutoTopPivot(m_topPivot, () -> m_drivetrain.getTrigToScoreInSpeaker()),
										new KeepBasePivotToAngle(m_basePivot, 90),
										new TurnToAngle(m_drivetrain, () -> m_drivetrain.getAngleToFaceSpeaker()),
										new SequentialCommandGroup(
												new ShootToRPM(m_shooter),
												new FeedShot(m_intake),
												new WaitCommand(.3),
												new StopIntake(m_intake),
												new StopShooter(m_shooter)))),
						Map.entry(CommandSelector.EIGHT,
								new ParallelRaceGroup(
										new AutoTopPivot(m_topPivot, () -> m_drivetrain.getTrigToScoreInSpeaker()),
										new KeepBasePivotToAngle(m_basePivot, 90),
										new TurnToAngle(m_drivetrain, () -> m_drivetrain.getAngleToFaceSpeaker()),
										new SequentialCommandGroup(
												new ShootToRPM(m_shooter),
												new FeedShot(m_intake),
												new WaitCommand(.3),
												new StopIntake(m_intake),
												new StopShooter(m_shooter))))

				),

				this::select);
		// runs method below to configure button bindings
		configureBindings();
		configureAutos();

		// add auton options in the constructor as it only happens once
		// m_autoChooser.addOption("none", Autos.none());

		m_drivetrain.setDefaultCommand(
				new SwerveDrive(
						m_drivetrain,
						() -> -testing.getRawAxis(translationAxis),
						() -> -testing.getRawAxis(strafeAxis),
						() -> -testing.getRawAxis(rotationAxis),
						() -> false));

		m_basePivot.setDefaultCommand(
				new ManualBasePivot(m_basePivot, () -> -operator.getRawAxis(1)));
		m_topPivot.setDefaultCommand(
				new ManualTopPivot(m_topPivot, () -> -operator.getRawAxis(5)));

	}

	private void configureBindings() {

		// new JoystickButton(operator, 1).onTrue(new Shoot(m_shooter));

		new JoystickButton(testing, 14).onTrue(

				new ConditionalCommand(
						new SequentialCommandGroup(
								new SetLeds(m_candle, Candle.LEDState.YELLOW),
								m_selectCommand,
								new SetLeds(m_candle, Candle.LEDState.GREEN)),

						new InstantCommand(),

						m_limelight::hasTargetAprilTag));

		new JoystickButton(testing, 10).onTrue(new InstantCommand(m_drivetrain::zeroGyro));

		// new JoystickButton(testing, 7).whileTrue(new OutTaking(m_intake));

		new JoystickButton(testing, 13).onTrue(
				new ConditionalCommand(
						new SequentialCommandGroup(
								new ConditionalCommand(
										new SequentialCommandGroup(new ShootToRPM(m_shooter)),
										new StopShooter(m_shooter),
										m_intake::isNotePresent),
								new ConditionalCommand(
										new SequentialCommandGroup(new FeedShot(m_intake), new WaitCommand(.6)),
										new StopIntake(m_intake),
										m_intake::isNotePresent),
								new ConditionalCommand(new FeedShot(m_intake),
										new StopIntake(m_intake),
										m_intake::isNotePresent),
								new ConditionalCommand(new ShootToRPM(m_shooter),
										new StopShooter(m_shooter),
										m_intake::isNotePresent)),

						new ConditionalCommand(new ShootToRPM(m_shooter),
								new StopShooter(m_shooter),
								m_shooter::shooterNotRunning),

						m_intake::colorSensorConnected));

		// button to ground intake
		new JoystickButton(testing, 1).onTrue(
				new ConditionalCommand(
						new SequentialCommandGroup(
								new SetTopPivotToAngle(m_topPivot, 36),
								new SetBasePivotToAngle(m_basePivot, -15),
								new Intaking(m_intake),
								new SetBasePivotToAngle(m_basePivot, 90)),

						new SequentialCommandGroup(
								new SetTopPivotToAngle(m_topPivot, 36),
								new SetBasePivotToAngle(m_basePivot, -15

								)),

						m_intake::colorSensorConnected));

		// sets angle to infront of speaker
		new JoystickButton(testing, 2).onTrue(
				new SequentialCommandGroup(new SetBasePivotToAngle(m_basePivot, 90),
						new SetTopPivotToAngle(m_topPivot, -50)));

		// Button to Score in Amp
		new JoystickButton(testing, 3).onTrue(
				new SequentialCommandGroup(
						new ParallelRaceGroup(
								new KeepBasePivotToAngle(m_basePivot, 22),
								new KeepTopPivotToAngle(m_topPivot, 153)))); // ,
		// new SequentialCommandGroup(
		// new WaitCommand(2),
		// new ParallelRaceGroup(
		// new WaitCommand(2),
		// new OutTaking(m_intake)))),

		// new SetBasePivotToAngle(m_basePivot, 90),
		// new SetTopPivotToAngle(m_topPivot, 5)));

		// button for source intake
		new JoystickButton(testing, 4).onTrue(
				new SequentialCommandGroup(
						new SetTopPivotToAngle(m_topPivot, 87),
						new SetBasePivotToAngle(m_basePivot, 127),
						new Intaking(m_intake), new SetBasePivotToAngle(m_basePivot, 90)

				));

		new JoystickButton(testing, 7).whileTrue(
				new OutTaking(m_intake));

		new JoystickButton(testing, 8).whileTrue(
				new Intaking(m_intake));

		new JoystickButton(driver, 7).whileTrue(
				new ClimbDown(m_climb));

		new JoystickButton(driver, 8).whileTrue(
				new ClimbUp(m_climb));

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
		m_autoChooser.addOption("4piece center", AutoBuilder.buildAuto("4 Piece Center"));

	}

	// The enum used as keys for selecting the Alignment command to run.
	private enum CommandSelector {
		ZERO,
		ONE,
		TWO,
		THREE,
		FOUR,
		FIVE,
		SIX,
		SEVEN,
		EIGHT,
		NINE,
		TEN,
		ELEVEN,
		TWELVE,
		THIRTEEN,
		FOURTEEN,
		FIFTEEN,
		SIXTEEN,
		SEVENTEEN,
		EIGHTTEEN,
		NINETEEN,
		TWENTY

	}

	// An example selector method for the selectcommand. Returns the selector that
	// will select
	// which command to run. Can base this choice on logical conditions evaluated at
	// runtime.
	private CommandSelector select() {
		switch ((int) m_limelight.getFid()) {
			case 1:
				return CommandSelector.ONE;
			case 2:
				return CommandSelector.TWO;
			case 3:
				return CommandSelector.THREE;
			case 4:
				return CommandSelector.FOUR;
			case 5:
				return CommandSelector.FIVE;
			case 6:
				return CommandSelector.SIX;
			case 7:
				return CommandSelector.SEVEN;
			case 8:
				return CommandSelector.EIGHT;
			case 9:
				return CommandSelector.NINE;
			case 10:
				return CommandSelector.TEN;
			case 11:
				return CommandSelector.ELEVEN;
			case 12:
				return CommandSelector.TWELVE;
			case 13:
				return CommandSelector.THIRTEEN;
			case 14:
				return CommandSelector.FOURTEEN;
			case 15:
				return CommandSelector.FIFTEEN;
			case 16:
				return CommandSelector.SIXTEEN;
			case 17:
				return CommandSelector.SEVENTEEN;
			case 18:
				return CommandSelector.EIGHTTEEN;
			case 19:
				return CommandSelector.NINETEEN;
			case 20:
				return CommandSelector.TWENTY;

			default:
				return CommandSelector.ZERO;
		}

	}

}