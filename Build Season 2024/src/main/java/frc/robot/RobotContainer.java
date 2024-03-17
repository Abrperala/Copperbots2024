package frc.robot;

import frc.robot.subsystems.BasePivot;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.PortClimber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StarboardClimber;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TopPivot;
import frc.robot.subsystems.Candle.LEDState;

import java.util.Map;

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
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
	public static PortClimber m_portClimb;
	public static StarboardClimber m_starboardClimb;

	/* Controllers */
	private final PS4Controller driver = new PS4Controller(0);
	private final PS4Controller operator = new PS4Controller(1);
	private final PS4Controller testing = new PS4Controller(2);

	/* Drive Controls */
	private final int translationAxis = PS4Controller.Axis.kLeftY.value;
	private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
	private final int rotationAxis = PS4Controller.Axis.kRightX.value;

	private final JoystickButton isEvading = new JoystickButton(driver, 5);
	private final JoystickButton isRotatingFast = new JoystickButton(driver, 6);
	private final POVButton isLocked = new POVButton(driver, 180);

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
		m_portClimb = new PortClimber();
		m_starboardClimb = new StarboardClimber();

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
						Map.entry(CommandSelector.ZERO, new InstantCommand()),
						// Map.entry(CommandSelector.ONE,
						// new ParallelCommandGroup(
						// m_drivetrain.followPathCommandtoAT1(),
						// new SequentialCommandGroup(
						// new SetTopPivotToAngle(m_topPivot, 87),
						// new SetBasePivotToAngle(m_basePivot,
						// 127),
						// new Intaking(m_intake),
						// new SetBasePivotToAngle(m_basePivot,
						// 90),
						// new SetTopPivotToAngle(m_topPivot, 0)))),
						// Map.entry(CommandSelector.TWO,
						// new ParallelCommandGroup(
						// m_drivetrain.followPathCommandtoAT2(),
						// new SequentialCommandGroup(
						// new SetTopPivotToAngle(m_topPivot, 87),
						// new SetBasePivotToAngle(m_basePivot,
						// 127),
						// new Intaking(m_intake),
						// new SetBasePivotToAngle(m_basePivot,
						// 90),
						// new SetTopPivotToAngle(m_topPivot, 0)))),
						Map.entry(CommandSelector.THREE,
								new ParallelRaceGroup(
										new AutoTopPivot(m_topPivot, () -> m_drivetrain.getTrigToScoreInSpeaker()),
										new KeepBasePivotToAngle(m_basePivot, 90),
										new TurnToAngle(m_drivetrain, () -> m_drivetrain.getAngleToFaceSpeaker()),
										new SequentialCommandGroup(
												new ShootToRPM(m_shooter),
												new WaitCommand(.3),
												new FeedShot(m_intake),
												new WaitCommand(.5),
												new StopIntake(m_intake),
												new StopShooter(m_shooter)))),
						Map.entry(CommandSelector.FOUR,
								new ParallelRaceGroup(
										new AutoTopPivot(m_topPivot, () -> m_drivetrain.getTrigToScoreInSpeaker()),
										new KeepBasePivotToAngle(m_basePivot, 90),
										new TurnToAngle(m_drivetrain, () -> m_drivetrain.getAngleToFaceSpeaker()),
										new SequentialCommandGroup(
												new ShootToRPM(m_shooter),
												new WaitCommand(.3),
												new FeedShot(m_intake),
												new WaitCommand(.5),
												new StopIntake(m_intake),
												new StopShooter(m_shooter)))),
						// Map.entry(CommandSelector.SIX,
						// new ParallelCommandGroup(
						// m_drivetrain.followPathCommandtoAT6(),
						// new SequentialCommandGroup(
						// new SetTopPivotToAngle(m_topPivot, 80),
						// new SetBasePivotToAngle(m_basePivot, 138),
						// new ParallelRaceGroup(
						// new OutTaking(m_intake),
						// new WaitCommand(.3)),
						// new SetTopPivotToAngle(m_topPivot, 105),
						// new ParallelRaceGroup(
						// new OutTaking(m_intake),
						// new WaitCommand(1)),
						// new StopIntake(m_intake),
						// new SetBasePivotToAngle(m_basePivot, 90),
						// new SetTopPivotToAngle(m_topPivot, 0)))),
						Map.entry(CommandSelector.SEVEN,
								new ParallelRaceGroup(
										new AutoTopPivot(m_topPivot, () -> m_drivetrain.getTrigToScoreInSpeaker()),
										new KeepBasePivotToAngle(m_basePivot, 90),
										new TurnToAngle(m_drivetrain, () -> m_drivetrain.getAngleToFaceSpeaker()),
										new SequentialCommandGroup(
												new ShootToRPM(m_shooter),
												new WaitCommand(.3),
												new FeedShot(m_intake),
												new WaitCommand(.5),
												new StopIntake(m_intake),
												new StopShooter(m_shooter)))),
						Map.entry(CommandSelector.EIGHT,
								new ParallelRaceGroup(
										new AutoTopPivot(m_topPivot, () -> m_drivetrain.getTrigToScoreInSpeaker()),
										new KeepBasePivotToAngle(m_basePivot, 90),
										new TurnToAngle(m_drivetrain, () -> m_drivetrain.getAngleToFaceSpeaker()),
										new SequentialCommandGroup(
												new ShootToRPM(m_shooter),
												new WaitCommand(.3),
												new FeedShot(m_intake),
												new WaitCommand(.5),
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
						() -> driver.getRawAxis(translationAxis),
						() -> driver.getRawAxis(strafeAxis),
						() -> driver.getRawAxis(rotationAxis),
						() -> false,
						() -> isEvading.getAsBoolean(),
						() -> isLocked.getAsBoolean(),
						() -> isRotatingFast.getAsBoolean()));

		// m_basePivot.setDefaultCommand(
		// new ManualBasePivot(m_basePivot, () -> operator.getRawAxis(1)));
		m_topPivot.setDefaultCommand(
				new ConditionalCommand(
						new ConditionalCommand(
								new AutoTopPivot(m_topPivot, () -> m_drivetrain.getTrigToScoreInSpeaker()),
								new InstantCommand(),
								m_drivetrain::isInWing),
						new InstantCommand(),
						m_basePivot::isIntaking));

	}

	private void configureBindings() {

		// TODO: change buttons for comp

		new JoystickButton(operator, 1).onTrue(
				new ConditionalCommand(
						new SequentialCommandGroup(
								new SetLeds(m_candle, Candle.LEDState.RED),
								m_selectCommand,
								new SetLeds(m_candle, Candle.LEDState.GREEN)),

						new InstantCommand(),
						m_limelight::hasTargetAprilTag));

		new JoystickButton(driver, 10).onTrue(new InstantCommand(m_drivetrain::zeroGyro));

		// button to toggle shooter
		new JoystickButton(driver, 13).onTrue(
				new ParallelCommandGroup(
						new ShootToLob(m_shooter),
						new SetLeds(m_candle, LEDState.RED)));

		new JoystickButton(driver, 13).onFalse(
				new ParallelCommandGroup(
						new StopShooter(m_shooter),
						new SetLeds(m_candle, LEDState.GREEN)));

		new JoystickButton(operator, 2).onTrue(
				new ParallelCommandGroup(
						new ShootToRPM(m_shooter),
						new SetLeds(m_candle, LEDState.RED)));

		new JoystickButton(operator, 2).onFalse(
				new ParallelCommandGroup(
						new StopShooter(m_shooter),
						new SetLeds(m_candle, LEDState.GREEN)));

		// button to ground intake
		new JoystickButton(driver, 1).onTrue(
				new SequentialCommandGroup(
						new SetLeds(m_candle, Candle.LEDState.ORANGE),
						new SetTopPivotToAngle(m_topPivot, 36),
						new SetBasePivotToAngle(m_basePivot, -10),
						new Intaking(m_intake),
						new SetLeds(m_candle, Candle.LEDState.GREEN),
						new ParallelCommandGroup(
								new ParallelRaceGroup(
										new WaitCommand(.2),
										new FeedShot(m_intake)),
								new SequentialCommandGroup(
										new SetBasePivotToAngle(m_basePivot, 90),
										new SetTopPivotToAngle(m_topPivot, -52)))));

		// button for operator to set the arms to ground intake, used for traversing
		new POVButton(operator, 180).onTrue(
				new SequentialCommandGroup(
						new SetTopPivotToAngle(m_topPivot, 40),
						new SetBasePivotToAngle(m_basePivot, -10)));

		// sets angle to infront of speaker
		new POVButton(operator, 90).onTrue(
				new SequentialCommandGroup(
						new SetBasePivotToAngle(m_basePivot, 90),
						new SetLeds(m_candle, Candle.LEDState.GREEN),
						new SetTopPivotToAngle(m_topPivot, -52)));

		new JoystickButton(driver, 2).onTrue(
				new SequentialCommandGroup(
						new SetBasePivotToAngle(m_basePivot, 90),
						new SetTopPivotToAngle(m_topPivot, -52)));

		// Button to Score in Amp
		// top 21, bottom 66, to top 53, 135 bottom, 67 top, 122 bottom, 92 top
		new JoystickButton(driver, 3).onTrue(
				new SequentialCommandGroup(
						new SetLeds(m_candle, Candle.LEDState.YELLOW),
						new SetTopPivotToAngle(m_topPivot, 5),
						new SetBasePivotToAngle(m_basePivot, 70),
						new OutTaking(m_intake),
						new WaitCommand(.3),
						new SetTopPivotToAngle(m_topPivot, 23),
						new StopIntake(m_intake),
						new SetLeds(m_candle, Candle.LEDState.GREEN),
						new SetBasePivotToAngle(m_basePivot, 90),
						new SetTopPivotToAngle(m_topPivot, 0))

		);

		// TODO: find out why so slow
		new POVButton(operator, 270).onTrue(
				new SequentialCommandGroup(
						new SetLeds(m_candle, Candle.LEDState.YELLOW),
						new SetTopPivotToAngle(m_topPivot, 5),
						new SetBasePivotToAngle(m_basePivot, 70),
						new OutTaking(m_intake),
						new WaitCommand(.3),
						new SetTopPivotToAngle(m_topPivot, 23),
						new StopIntake(m_intake),
						new SetLeds(m_candle, Candle.LEDState.GREEN),
						new SetBasePivotToAngle(m_basePivot, 90),
						new SetTopPivotToAngle(m_topPivot, 0))

		);

		// button for backup source intake+
		new JoystickButton(driver, 4).onTrue(
				new SequentialCommandGroup(
						new SetTopPivotToAngle(m_topPivot, 85),
						new SetBasePivotToAngle(m_basePivot, 60)));

		// button for source intake top 85, bottom 60
		new POVButton(operator, 0).onTrue(
				new SequentialCommandGroup(
						new SetLeds(m_candle, Candle.LEDState.ORANGE),
						new ParallelCommandGroup(
								new SetTopPivotToAngle(m_topPivot, 85),
								new SetBasePivotToAngle(m_basePivot, 60)),
						new Intaking(m_intake),
						new SetLeds(m_candle, Candle.LEDState.GREEN),
						new ParallelCommandGroup(
								new ParallelRaceGroup(
										new WaitCommand(.2),
										new FeedShot(m_intake)),
								new ParallelCommandGroup(
										new SetBasePivotToAngle(m_basePivot, 90),
										new SetTopPivotToAngle(m_topPivot, -52)))));

		new JoystickButton(driver, 7).whileTrue(
				new ParallelCommandGroup(
						new OutTaking(m_intake),
						new SetLeds(m_candle, LEDState.BLUE)));

		new JoystickButton(driver, 7).onFalse(
				new ParallelCommandGroup(
						new StopIntake(m_intake),
						new SetLeds(m_candle, LEDState.GREEN)));

		new JoystickButton(driver, 8).whileTrue(
				new ParallelCommandGroup(
						new SetLeds(m_candle, Candle.LEDState.ORANGE),
						new ManualIntaking(m_intake)));

		new JoystickButton(driver, 8).onFalse(
				new SetLeds(m_candle, Candle.LEDState.GREEN));

		new JoystickButton(operator, 6).whileTrue(
				new StarboardClimb(m_starboardClimb, -.5));

		new JoystickButton(operator, 5).whileTrue(
				new PortClimb(m_portClimb, -.5));

		new JoystickButton(operator, 8).whileTrue(
				new StarboardClimb(m_starboardClimb, .5));

		new JoystickButton(operator, 7).whileTrue(
				new PortClimb(m_portClimb, .5));

	}

	private void configureAutos() {
		NamedCommands.registerCommand("AngleForSubwooferShot",
				new SequentialCommandGroup(
						new SetBasePivotToAngle(m_basePivot, 90),
						new SetTopPivotToAngle(m_topPivot, -52)));
		NamedCommands.registerCommand("Shoot",
				new SequentialCommandGroup(
						new ConditionalCommand(
								new SequentialCommandGroup(
										new ShootToRPM(m_shooter)),
								new StopShooter(m_shooter),
								m_intake::isNotePresent),
						new ConditionalCommand(
								new SequentialCommandGroup(
										new FeedShot(m_intake),
										new WaitCommand(.6)),
								new StopIntake(m_intake),
								m_intake::isNotePresent),
						new ConditionalCommand(
								new FeedShot(m_intake),
								new StopIntake(m_intake),
								m_intake::isNotePresent),
						new ConditionalCommand(
								new ShootToRPM(m_shooter),
								new StopShooter(m_shooter),
								m_intake::isNotePresent)));
		NamedCommands.registerCommand("Start Shooter", new ShootToRPM(m_shooter));
		NamedCommands.registerCommand("Feed Shot", new FeedShot(m_intake));
		NamedCommands.registerCommand("Stop Shooter", new StopShooter(m_shooter));
		NamedCommands.registerCommand("Intake",
				new Intaking(m_intake));
		NamedCommands.registerCommand("Arm To Intake",
				new SequentialCommandGroup(
						new SetTopPivotToAngle(m_topPivot, 40),
						new SetBasePivotToAngle(m_basePivot, -10)));
		NamedCommands.registerCommand("Arm To Intake Shot",
				new SequentialCommandGroup(
						new KeepTopPivotToAngle(m_topPivot, 55)));
		NamedCommands.registerCommand("Top Arm To Intake",
				new SetTopPivotToAngle(m_topPivot, 40));
		NamedCommands.registerCommand("Base Arm To Intake",
				new SetBasePivotToAngle(m_basePivot, -10));

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

	}

	public void addAutonOptions() {
		m_autoChooser.setDefaultOption("none", new InstantCommand() {
		});
		m_autoChooser.addOption("1 piece center", AutoBuilder.buildAuto("1 Piece Center"));
		m_autoChooser.addOption("5 piece center", AutoBuilder.buildAuto("5 Piece Center"));
		m_autoChooser.addOption("4 piece center", AutoBuilder.buildAuto("4 Piece Center"));
		m_autoChooser.addOption("3 Piece Source", AutoBuilder.buildAuto("3 Piece Source"));
		m_autoChooser.addOption("Shoot Only", new SequentialCommandGroup(
				new SetBasePivotToAngle(m_basePivot, 90),
				new SetTopPivotToAngle(m_topPivot, -52),
				new ShootToRPM(m_shooter),
				new FeedShot(m_intake),
				new StopShooter(m_shooter)));
		m_autoChooser.addOption("1 Piece Source", AutoBuilder.buildAuto("1 Piece Source"));
		m_autoChooser.addOption("BB 5 Piece Center", AutoBuilder.buildAuto("BB 5 Piece Center"));
		m_autoChooser.addOption("3 Piece Amp", AutoBuilder.buildAuto("3 Piece Amp"));

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