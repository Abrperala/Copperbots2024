package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.DriveToPose;
import frc.robot.Commands.IntakeFeed;
import frc.robot.Commands.IntakeNote;
import frc.robot.Commands.IntakeUntilTripped;
import frc.robot.Commands.SeekNote;
import frc.robot.Commands.SetArmState;
import frc.robot.Commands.SetClimbState;
import frc.robot.Commands.SetShooterState;
import frc.robot.Commands.SetWristState;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Arm.ArmState;
import frc.robot.Subsystems.Climb.ClimbState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Wrist.WristState;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        private double MaxAngularRate = 1.5 * Math.PI;

        private final PS4Controller driver = new PS4Controller(0);
        private final PS4Controller testing = new PS4Controller(2);

        private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
        private final Wrist wrist = new Wrist();
        private final Arm arm = new Arm();
        private final Intake intake = new Intake();
        private final Shooter shooter = new Shooter();
        private final Climb portClimb = new Climb(22);
        private final Climb starboardClimb = new Climb(21);
        private final Vision vision = new Vision();

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final JoystickButton driverSquareButton = new JoystickButton(driver, 1);
        private final JoystickButton driverCrossButton = new JoystickButton(driver, 2);
        private final JoystickButton driverCircle = new JoystickButton(driver, 3);
        private final JoystickButton driverTriangle = new JoystickButton(driver, 4);
        private final JoystickButton driverLeftBumper = new JoystickButton(driver, 5);
        private final JoystickButton driverRightBumper = new JoystickButton(driver, 6);
        private final JoystickButton driverLeftTriggerButton = new JoystickButton(driver, 7);
        private final JoystickButton driverRightTriggerButton = new JoystickButton(driver, 8);
        private final JoystickButton driverTouchpad = new JoystickButton(driver, 14);
        private final JoystickButton driverOptionsButton = new JoystickButton(testing, 10);

        private final POVButton driverTopPov = new POVButton(driver, 0);
        private final POVButton driverTopRightPov = new POVButton(driver, 45);
        private final POVButton driverRightPov = new POVButton(driver, 90);
        private final POVButton driverRightBottomPov = new POVButton(driver, 135);
        private final POVButton driverBottomPov = new POVButton(driver, 180);
        private final POVButton driverLeftbottomPov = new POVButton(driver, 225);
        private final POVButton driverLeftPov = new POVButton(driver, 270);
        private final POVButton driverLeftTopPov = new POVButton(driver, 315);

        private final JoystickButton testingSquareButton = new JoystickButton(testing, 1);
        private final JoystickButton testingCrossButton = new JoystickButton(testing, 2);
        private final JoystickButton testingCircleButton = new JoystickButton(testing, 3);
        private final JoystickButton testingTriangle = new JoystickButton(testing, 4);
        private final JoystickButton testingLeftBumper = new JoystickButton(testing, 5);
        private final JoystickButton testingRightBumper = new JoystickButton(testing, 6);
        private final JoystickButton testingLeftTriggerButton = new JoystickButton(testing, 7);
        private final JoystickButton testingRightTriggerButton = new JoystickButton(testing, 8);
        private final JoystickButton testingTouchpad = new JoystickButton(testing, 14);

        private Command runAuto = drivetrain.getAutoPath("test");

        public RobotContainer() {
                configureBindings();
        }

        private void configureBindings() {

                final Command groundIntaking = new SequentialCommandGroup(new SetWristState(wrist, WristState.Intaking),
                                new WaitCommand(.1), new SetArmState(arm, ArmState.Intaking),
                                new IntakeUntilTripped(intake), new SetArmState(arm, ArmState.Standby),
                                new WaitCommand(.3), new SetWristState(wrist, WristState.Standby));

                final Command standbyArmAndWrist = new SequentialCommandGroup(new SetArmState(arm, ArmState.Standby),
                                new WaitCommand(.3), new SetWristState(wrist, WristState.Standby));

                final Command amping = new SequentialCommandGroup(new IntakeUntilTripped(intake),
                                new IntakeUntilTripped(intake),
                                new ParallelCommandGroup(new SetShooterState(shooter, ShooterState.Amping),
                                                new SetArmState(arm, ArmState.Amping),
                                                new SetWristState(wrist, WristState.Amping)),
                                new WaitCommand(.4),
                                new IntakeFeed(intake),
                                new WaitCommand(.2),
                                new ParallelCommandGroup(new SetShooterState(shooter, ShooterState.Standby),
                                                new SetArmState(arm, ArmState.Standby),
                                                new SetWristState(wrist, WristState.Standby)));

                final Command sourceIntaking = new SequentialCommandGroup(
                                new ParallelCommandGroup(new SetArmState(arm, ArmState.Sourcing),
                                                new SetWristState(wrist, WristState.Sourcing),
                                                new IntakeUntilTripped(intake)),
                                new ParallelCommandGroup(new SetArmState(arm, ArmState.Standby),
                                                new SetWristState(wrist, WristState.Standby)));

                final Command speakerShot = new SequentialCommandGroup(
                                new ParallelCommandGroup(new SetShooterState(shooter, ShooterState.Shooting),
                                                new SetArmState(arm, ArmState.Standby),
                                                new SetWristState(wrist, WristState.SpeakerShot)),
                                new WaitCommand(.4),
                                new IntakeFeed(intake),
                                new WaitCommand(.2),
                                new ParallelCommandGroup(new SetShooterState(shooter, ShooterState.Standby),
                                                new SetArmState(arm, ArmState.Standby),
                                                new SetWristState(wrist, WristState.Standby)));

                final Command portClimbUp = new SetClimbState(portClimb, ClimbState.ClimbUp);

                final Command portClimbDown = new SetClimbState(portClimb, ClimbState.ClimbDown);

                final Command portClimbStandby = new SetClimbState(portClimb, ClimbState.Standby);

                final Command starboardClimbUp = new SetClimbState(starboardClimb, ClimbState.ClimbUp);

                final Command starboardClimbDown = new SetClimbState(starboardClimb, ClimbState.ClimbDown);

                final Command starboardClimbStandby = new SetClimbState(starboardClimb, ClimbState.Standby);

                testingLeftTriggerButton.onTrue(portClimbUp);

                testingLeftTriggerButton.onFalse(portClimbStandby);

                testingLeftBumper.onTrue(portClimbDown);

                testingLeftBumper.onFalse(portClimbStandby);

                testingRightTriggerButton.onTrue(starboardClimbUp);

                testingRightTriggerButton.onFalse(starboardClimbStandby);

                testingRightBumper.onTrue(starboardClimbDown);

                testingRightBumper.onFalse(starboardClimbStandby);

                testingCrossButton.onTrue(sourceIntaking);

                testingTouchpad.onTrue(speakerShot);

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-(testing.getLeftY() * testing.getLeftY()
                                                                * Math.signum(testing.getLeftY())) * MaxSpeed)
                                                .withVelocityY(-(testing.getLeftX() * testing.getLeftX()
                                                                * Math.signum(testing.getLeftX())) * MaxSpeed)
                                                .withRotationalRate(-(testing.getRightX() * testing.getRightX()
                                                                * Math.signum(testing.getRightX())) * MaxAngularRate)));

                testingCircleButton.onTrue(standbyArmAndWrist);

                testingSquareButton.onTrue(groundIntaking);

                testingTriangle.onTrue(amping);

                driverCrossButton.whileTrue(drivetrain.applyRequest(() -> brake));

                driverSquareButton.whileTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
                driverOptionsButton.onTrue(
                                new InstantCommand(drivetrain::tareEverything));

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}
