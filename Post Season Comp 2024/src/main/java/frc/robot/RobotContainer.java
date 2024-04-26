package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        private double MaxAngularRate = 1.5 * Math.PI;

        private final PS4Controller driver = new PS4Controller(0);
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
        private final Wrist wrist = new Wrist();
        private final Arm arm = new Arm();
        private final Intake intake = new Intake();
        private final Shooter shooter = new Shooter();
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

        private final POVButton driverTopPov = new POVButton(driver, 0);
        private final POVButton driverTopRightPov = new POVButton(driver, 45);
        private final POVButton driverRightPov = new POVButton(driver, 90);
        private final POVButton driverRightBottomPov = new POVButton(driver, 135);
        private final POVButton driverBottomPov = new POVButton(driver, 180);
        private final POVButton driverLeftbottomPov = new POVButton(driver, 225);
        private final POVButton driverLeftPov = new POVButton(driver, 270);
        private final POVButton driverLeftTopPov = new POVButton(driver, 315);

        private void configureBindings() {

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                                                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverTopPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(.5 * MaxSpeed)
                                                .withVelocityY(0 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverRightPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(.0 * MaxSpeed)
                                                .withVelocityY(-.5 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverBottomPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * MaxSpeed)
                                                .withVelocityY(.0 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverLeftPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(.0 * MaxSpeed)
                                                .withVelocityY(.5 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverTopRightPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(.5 * MaxSpeed)
                                                .withVelocityY(-.5 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverRightBottomPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * MaxSpeed)
                                                .withVelocityY(-.5 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverLeftbottomPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * MaxSpeed)
                                                .withVelocityY(.5 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverLeftTopPov.whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(.5 * MaxSpeed)
                                                .withVelocityY(.5 * MaxSpeed)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

                driverLeftTriggerButton.whileTrue(
                                new InstantCommand(wrist::turnClockwise));

                driverRightTriggerButton.whileTrue(
                                new InstantCommand(wrist::turnCounterClockwise));

                driverLeftTriggerButton.onFalse(
                                new InstantCommand(wrist::stopTurning));

                driverRightTriggerButton.onFalse(
                                new InstantCommand(wrist::stopTurning));

                driverRightBumper.whileTrue(
                                new InstantCommand(arm::turnClockwise));

                driverLeftBumper.whileTrue(
                                new InstantCommand(arm::turnCounterClockwise));

                driverRightBumper.onFalse(
                                new InstantCommand(arm::stopTurning));

                driverLeftBumper.onFalse(
                                new InstantCommand(arm::stopTurning));

                driverTriangle.onTrue(
                                new InstantCommand(() -> intake.setIntakeState(IntakeState.Intaking)));

                driverTriangle.onFalse(
                                new InstantCommand(() -> intake.setIntakeState(IntakeState.Standby)));

                // driverCircle.onFalse(
                // new InstantCommand(intake::stopIntake));

                // driverCircle.whileTrue(
                // new InstantCommand(intake::outtaking));

                driverTouchpad.whileTrue(
                                new InstantCommand(() -> shooter.setShooterState(ShooterState.Amping)));

                driverTouchpad.onFalse(
                                new InstantCommand(() -> shooter.setShooterState(ShooterState.Standby)));

                driverCrossButton.whileTrue(drivetrain.applyRequest(() -> brake));
                driverSquareButton.whileTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public RobotContainer() {
                configureBindings();
        }

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}
