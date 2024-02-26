package frc.robot.subsystems;

import frc.lib.util.CopperBotUtils;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveMods;
    public AHRS m_gyro;
    public AutoBuilder autoBuilder;
    private Field2d field;

    public SwerveDrivetrain() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        while (m_gyro.isCalibrating())
            ;
        Timer.delay(5);
        m_gyro.reset();
        System.out.println("NavX MXP has been reset!");

        autoBuilder = new AutoBuilder();

        m_swerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Mod0.constants),
                new SwerveModule(1, Constants.Mod1.constants),
                new SwerveModule(2, Constants.Mod2.constants),
                new SwerveModule(3, Constants.Mod3.constants)
        };

        m_swerveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getModulePositions());

        setOdometryForOdometryAlign();

        // Auto stuffs

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(.02, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(.07, 0.0, 0.0), // Rotation PID constants

                        Constants.MAX_SPEED, // Max module speed, in m/s
                        Constants.DRIVEBASE_RADIUS, // Drive base radius in meters. Distance from robot center to
                                                    // furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        field = new Field2d();
        SmartDashboard.putData("Field", field);

    }

    /**
     * Drive method for swerve drivetrain.
     * 
     * @param translation   The desired translation vector.
     * @param rotation      The desired rotation value.
     * @param fieldRelative Whether the translation is field relative.
     * @param isOpenLoop    Whether the drive is open loop.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        m_gyro.getRotation2d()) // drive based on gyro

                        // getPose().getRotation()) // drive based on pose based on gyro?
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }

    /**
     * Set chassis speeds for the swerve drivetrain.
     * 
     * @param targetSpeeds The target chassis speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(Constants.swerveKinematics.toSwerveModuleStates(targetSpeeds));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Set desired states for each swerve module.
     * 
     * @param desiredStates The desired states for each swerve module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    /**
     * Gets the current robot pose in the field coordinate system.
     * 
     * @return The current robot pose.
     */
    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a specified pose.
     * 
     * @param pose The desired pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        System.out.println(pose.getX());
        System.out.println(pose.getY());
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void setOdometryToOffset() {
        m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(0.0), getModulePositions(),
                new Pose2d(-6.14, 1.21, Rotation2d.fromDegrees(0.0)));
    }

    public void setOdometryForOdometryAlign() {
        m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(0.0), getModulePositions(),
                new Pose2d(13.56, 5.2, Rotation2d.fromDegrees(0.0)));
    }

    /**
     * Gets the current state of each swerve module.
     * 
     * @return Array of SwerveModuleState representing the current state of each
     *         swerve module.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveMods) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Gets the current position of each swerve module.
     * 
     * @return Array of SwerveModulePosition representing the current position of
     *         each swerve module.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveMods) {
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Stops the swerve drivetrain.
     */
    public void stopSwerve() {
        Translation2d stop = new Translation2d(0, 0);
        drive(stop, 0, true, true);

    }

    /**
     * Zeros the gyro by resetting its accumulated yaw.
     */
    public void zeroGyro() {
        m_gyro.reset();
        resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(0)));
    }

    /**
     * Gets the current yaw (rotation around Z-axis) from the gyro.
     * 
     * @return The current yaw as a Rotation2d.
     */
    public Rotation2d getYaw() {
        return m_gyro.getRotation2d();

    }

    public Command followPathCommand(int aprilTagId) {
        if (!hasTargetAprilTag(aprilTagId)) {
            return new WaitCommand(0.1);
        } else {

            Pose2d target = getTargetPoseFromAlliance(aprilTagId);

            return AutoBuilder.pathfindToPose(target, new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), 0);
        }

    }

    public Pose2d getTargetPoseFromAlliance(int aprilTagID) {
        if (CopperBotUtils.isAllianceBlue()) {
            switch (aprilTagID) {
                case 6:
                    return Constants.BLUE_AMP_SCORING_POSITION;
                case 1:
                    return Constants.BLUE_CLOSE_SOURCE_POSITION;
                case 2:
                    return Constants.BLUE_FAR_SOURCE_POSITION;
                default:
                    return null;
            }
        } else {
            switch (aprilTagID) {
                case 5:
                    return Constants.RED_AMP_SCORING_POSITION;
                case 9:
                    return Constants.RED_FAR_SOURCE_POSITION;
                case 10:
                    return Constants.RED_CLOSE_SOURCE_POSITION;
                default:
                    return null;
            }
        }
    }

    public boolean hasTargetAprilTag(int aprilTagID) {
        if (CopperBotUtils.isAllianceBlue()) {
            if (aprilTagID == 1 || aprilTagID == 2 || aprilTagID == 6) {
                return true;
            } else {
                return false;
            }
        } else {
            if (aprilTagID == 5 || aprilTagID == 9 || aprilTagID == 10) {
                return true;
            } else {
                return false;
            }
        }
    }

    @Override
    public void periodic() {
        m_swerveOdometry.update(getYaw(), getModulePositions());
        field.setRobotPose(getPose());

        for (SwerveModule mod : m_swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("real robot pose x", getPose().getX());
        SmartDashboard.putNumber("real robot pose y", getPose().getY());
        SmartDashboard.putNumber("real robot pose rot", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Rot", getYaw().getDegrees());

        SmartDashboard.putBoolean("is Alliance Blue?", CopperBotUtils.isAllianceBlue());

    }
}