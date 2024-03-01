// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
                public static final int DRIVE_MOTOR_ID = 8;
                public static final int ANGLE_MOTOR_ID = 7;
                public static final int CANCODER_ID = 12;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(143.701172);

                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
                public static final int DRIVE_MOTOR_ID = 4;
                public static final int ANGLE_MOTOR_ID = 3;
                public static final int CANCODER_ID = 10;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(165.498047);

                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
                public static final int DRIVE_MOTOR_ID = 6;
                public static final int ANGLE_MOTOR_ID = 5;
                public static final int CANCODER_ID = 11;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-41.923828);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        // TODO: figure out why BR module is 90 degrees off sometimes

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
                public static final int DRIVE_MOTOR_ID = 2;
                public static final int ANGLE_MOTOR_ID = 1;
                public static final int CANCODER_ID = 9;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(47.197266);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        /* Gyro reversed */
        public static final boolean INVERT_GYRO = false;

        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* Angle Motor PID Values */
        public static final double AZIMUTH_P = 7.2;
        public static final double AZIMUTH_I = 0.0;
        public static final double AZIMUTH_D = 0.1;
        public static final double AZIMUTH_F = 0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_S = (0.32);
        public static final double DRIVE_V = (1.51);
        public static final double DRIVE_A = (0.27);

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.12; // 0.1
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0;

        /* Swerve Current Limiting */
        public static final int AZIMUTH_CURRENT_LIMIT = 25;
        public static final int AZIMUTH_CURRENT_THRESHOLD = 40;
        public static final double AZIMUTH_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 35;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Neutral Modes */
        public static final NeutralModeValue AZIMUTH_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Swerve Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);
        public static final double AZIMUTH_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = (Units.feetToMeters(16.2)); // meters per second (theoretical from SDS)
        public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12 * 0.5; // radians per second (theoretical
                                                                                // calculation)
        public static final double TURN_IN_PLACE_SPEED = 0.5;
        public static final double A_RATE_LIMITER = 2.0; // Slew Rate Limiter Constant

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                        Math.PI, (Math.PI * Math.PI));

        public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(27);
        public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(27);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double DRIVEBASE_RADIUS = Units
                        .inchesToMeters(.5 * Math.sqrt(2 * Math.pow(DRIVETRAIN_LENGTH, 2)));

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                        new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
                        new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
                        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
                        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));

        public static final double STICK_DEADBAND = 0.1;
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final double AUTO_P_X_CONTROLLER = 0.1; // 0.1 for auto
        public static final double AUTO_P_Y_CONTROLLER = 0.1; // 1.4884 for auto
        public static final double AUTO_P_THETA_CONTROLLER = 1.375; // 2.8 for auto
        public static final double AUTO_I_THETA_CONTROLLER = 0;
        public static final double AUTO_D_THETA_CONTROLLER = 0.0;
        public static final double AUTO_MAX_SPEED = Units.feetToMeters(4.9);
        public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 3;

        public static final TrapezoidProfile.Constraints X_AUTO_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
        public static final TrapezoidProfile.Constraints Y_AUTO_CONSTRAINTS = new TrapezoidProfile.Constraints(0.25,
                        0.5);
        public static final TrapezoidProfile.Constraints THETA_AUTO_CONSTRAINTS = new TrapezoidProfile.Constraints(8,
                        8);

        public static final ProfiledPIDController AUTO_X_CONTROLLER = new ProfiledPIDController(AUTO_P_X_CONTROLLER, 0,
                        0,
                        X_AUTO_CONSTRAINTS);
        public static final ProfiledPIDController AUTO_Y_CONTROLLER = new ProfiledPIDController(AUTO_P_Y_CONTROLLER, 0,
                        0,
                        Y_AUTO_CONSTRAINTS);
        public static final ProfiledPIDController AUTO_THETA_CONTROLLER = new ProfiledPIDController(
                        AUTO_P_THETA_CONTROLLER,
                        AUTO_I_THETA_CONTROLLER, AUTO_D_THETA_CONTROLLER, THETA_AUTO_CONSTRAINTS);

        // Pivot Constants
        public static final int BASE1_PIVOT_ID = 16;
        public static final int BASE2_PIVOT_ID = 17;
        public static final int TOP_PIVOT_ID = 18;

        // LimeLight Constants, not actually used in code, but used in Limelight Configs

        /*
         * Front LL:
         * Forward: .1016
         * Right: -.2921
         * Up: .2819
         * 
         * Rear LL:
         * 
         * 
         */
        // values from Team Spectrum 3847’s X-Ray robot from last year
        // https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/5
        public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.1, 0.1, 10);

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision less. This matrix is in the form
         * [x, y, theta]ᵀ, with units in meters and radians.
         */

        // values from Team Spectrum 3847’s X-Ray robot from last year
        public static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, 500);
        // field elements positions constants

        /*
         * Posisitions of Field Elements
         * / Blue Amp: 1.85, 7.60, 90
         * / Red Amp:
         * /
         */

        public static final double HEIGHT_TO_SPEAKER_TARGET = 89;
        public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(0.076, 5.547868, new Rotation2d());
        public static final Pose2d RED_SPEAKER_POSE = new Pose2d(16.465042, 5.547868, new Rotation2d());

        public static final Pose2d BLUE_AMP_SCORING_POSITION = new Pose2d(1.85, 7.60,
                        new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d RED_AMP_SCORING_POSITION = new Pose2d(1.85, 0.5,
                        new Rotation2d(Math.toRadians(90)));
        // apriltag 2
        public static final Pose2d BLUE_FAR_SOURCE_POSITION = new Pose2d(15.8, 1.18,
                        new Rotation2d(Math.toRadians(120)));
        // apriltag 1
        public static final Pose2d BLUE_CLOSE_SOURCE_POSITION = new Pose2d(14.85, .55,
                        new Rotation2d(Math.toRadians(120)));
        // apriltag 9
        public static final Pose2d RED_FAR_SOURCE_POSITION = new Pose2d(16.1, 6.7,
                        new Rotation2d(Math.toRadians(-45)));
        // apriltag 10
        public static final Pose2d RED_CLOSE_SOURCE_POSITION = new Pose2d(14.8, 7.33,
                        new Rotation2d(Math.toRadians(-45)));

        // Shooter Constants
        public static final int SHOOT1_ID = 13;
        public static final int SHOOT2_ID = 14;
        public static final int INTAKE_ID = 15;

        public static double SHOOTER_GEARING = 1.5 / 1;
        public static double SHOOTER_TARGET_RPM = 6300;
        public static double SHOOTER_FREE_RPM = 5300 * SHOOTER_GEARING;

        // Pivot Constants

        public static final double HEIGHT_FROM_FLOOR_TO_1ST_PIVOT = 13.25;
        public static final double LENGTH_FROM_1ST_PIVOT_TO_2ND_PIVOT = 19;
        public static final double MAX_TOP_PIVOT_ANGLE = 160;
        public static final double MIN_TOP_PIVOT_ANGLE = -60;
        public static final double MAX_BOTTOM_PIVOT_ANGLE = 130;
        public static final double MIN_BOTTOM_PIVOT_ANGLE = -15;

        // LED Constants
        public static final int CANdleID = 20;

        public static final int PURPLE_R = 128;
        public static final int PURPLE_G = 0;
        public static final int PURPLE_B = 128;
        public static final int GREEN_R = 0;
        public static final int GREEN_G = 128;
        public static final int GREEN_B = 0;
        public static final int ORANGE_R = 255;
        public static final int ORANGE_G = 128;
        public static final int ORANGE_B = 0;
        public static final int RED_R = 255;
        public static final int RED_G = 0;
        public static final int RED_B = 0;
        public static final int YELLOW_R = 255;
        public static final int YELLOW_G = 118;
        public static final int YELLOW_B = 0;
        public static final int NONE_R = 0;
        public static final int NONE_G = 0;
        public static final int NONE_B = 0;
        public static final int WHITE_R = 255;
        public static final int WHITE_G = 255;
        public static final int WHITE_B = 255;
        public static final int COPPER_R = 184;
        public static final int COPPER_G = 115;
        public static final int COPPER_B = 51;

}
