// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.lib.util.SwerveModuleConstants;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
 

  /* Azimuth reversed */
  public static boolean FRONT_LEFT_AZIMUTH_REVERSED = false;
  public static boolean FRONT_RIGHT_AZIMUTH_REVERSED = false;
  public static boolean BACK_LEFT_AZIMUTH_REVERSED = false;
  public static boolean BACK_RIGHT_AZIMUTH_REVERSED = false;

  /* Drive motors reversed */
  public static boolean FRONT_LEFT_DRIVE_REVERSED = true;
  public static boolean FRONT_RIGHT_DRIVE_REVERSED = true;
  public static boolean BACK_LEFT_DRIVE_REVERSED = true;
  public static boolean BACK_RIGHT_DRIVE_REVERSED = true;

  /* CANCoders reversed */
  public static boolean FRONT_LEFT_CANCODER_REVERSED = false;
  public static boolean FRONT_RIGHT_CANCODER_REVERSED = false;
  public static boolean BACK_LEFT_CANCODER_REVERSED = false;
  public static boolean BACK_RIGHT_CANCODER_REVERSED = false;

  /* Module Specific Constants */
      /* Front Left Module - Module 0 */
      public static final class Mod0 {
          public static final int DRIVE_MOTOR_ID = 2;
          public static final int ANGLE_MOTOR_ID = 1;
          public static final int CANCODER_ID = 9;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(48.3 + 180);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 {
          public static final int DRIVE_MOTOR_ID = 4;
          public static final int ANGLE_MOTOR_ID = 3;
          public static final int CANCODER_ID = 10;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(160.4);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
      }
      
      /* Back Left Module - Module 2 */
      public static final class Mod2 {
          public static final int DRIVE_MOTOR_ID = 8;
          public static final int ANGLE_MOTOR_ID = 7;
          public static final int CANCODER_ID = 12;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(234.3 - 180);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3{
          public static final int DRIVE_MOTOR_ID = 6;
          public static final int ANGLE_MOTOR_ID = 5;
          public static final int CANCODER_ID = 11;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(226.2);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
      }

  /* Gyro reversed */
  public static final boolean INVERT_GYRO = false;

  /* Angle Motor PID Values */
  public static final double AZIMUTH_P = 0.2;
  public static final double AZIMUTH_I = 0.0;
  public static final double AZIMUTH_D = 0.1;
  public static final double AZIMUTH_F = 0;

  /* Drive Motor Characterization Values */
  public static final double DRIVE_S = (0.48665 / 12); //Values from SysId divided by 12 to convert to volts for CTRE
  public static final double DRIVE_V = (2.4132 / 12);
  public static final double DRIVE_A = (0.06921 / 12);

  /* Drive Motor PID Values */
  public static final double DRIVE_P = 0.1;  //0.1
  public static final double DRIVE_I = 0.0;
  public static final double DRIVE_D = 0.0;
  //public static final SimpleMotorFeedforward DRIVE_F = new SimpleMotorFeedforward(DRIVE_S, DRIVE_V, DRIVE_A);
  public static final double DRIVE_F = 0;

  /* Azimuth Current Limiting */
  public static final int AZIMUTH_CONTINUOUS_CURRENT_LIMIT = 25;
  public static final int AZIMUTH_PEAK_CURRENT_LIMIT = 40;
  public static final double AZIMUTH_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

  /* Drive Current Limiting */
  public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
  public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
  public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

  /* Neutral Modes */
  public static final NeutralMode AZIMUTH_NEUTRAL_MODE = NeutralMode.Coast;
  public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

  /* Swerve Gear Ratios */
  public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);
  public static final double AZIMUTH_GEAR_RATIO = (-150.0 / 7);

  /* Swerve Profiling Values */
  public static final double MAX_SPEED = (Units.feetToMeters(16.2)); //meters per second (theoretical from SDS)
  public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12*0.5; //radians per second (theoretical calculation)
  public static final double TURN_IN_PLACE_SPEED = 0.5;
  public static final double A_RATE_LIMITER = 2.0; //Slew Rate Limiter Constant
  
  /* Constraint for the motion profilied robot angle controller */
  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          Math.PI, (Math.PI * Math.PI));

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(21.75);
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(21.75);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));

    public static final double STICK_DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
}
