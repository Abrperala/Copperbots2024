package frc.robot.subsystems;

import frc.lib.util.GeneralUtils;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.List;
import java.util.Optional;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveModule[] m_swerveMods;
    public AHRS m_gyro;
    public AutoBuilder autoBuilder;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Limelight m_limeLight = Limelight.getInstance();

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

        poseEstimator = new SwerveDrivePoseEstimator(Constants.swerveKinematics,
                getYaw(), getModulePositions(),
                new Pose2d(0, 0, m_gyro.getRotation2d()), Constants.STATE_STDS, Constants.VISION_STDS);

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
                        // m_gyro.getRotation2d()) // drive based on gyro

                        getPose().getRotation()) // drive based on pose based on gyro?
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
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to a specified pose.
     * 
     * @param pose The desired pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        System.out.println(pose.getX());
        System.out.println(pose.getY());
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
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

    // Blue pathfinding commands
    public Command followPathCommandtoAT1() {
        Pose2d target = Constants.BLUE_CLOSE_SOURCE_POSITION;
        return AutoBuilder.pathfindToPose(target, new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), 0);
    }

    public Command followPathCommandtoAT2() {
        Pose2d target = Constants.BLUE_FAR_SOURCE_POSITION;
        return AutoBuilder.pathfindToPose(target, new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), 0);
    }

    public Command followPathCommandtoAT6() {
        Pose2d target = Constants.BLUE_AMP_SCORING_POSITION;
        return AutoBuilder.pathfindToPose(target, new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), 0);
    }

    // Red pathfinding commands
    public Command followPathCommandtoAT5() {
        Pose2d target = Constants.RED_AMP_SCORING_POSITION;
        return AutoBuilder.pathfindToPose(target, new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), 0);
    }

    public Command followPathCommandtoAT9() {
        Pose2d target = Constants.RED_FAR_SOURCE_POSITION;
        return AutoBuilder.pathfindToPose(target, new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), 0);
    }

    public Command followPathCommandtoAT10() {
        Pose2d target = Constants.RED_CLOSE_SOURCE_POSITION;
        return AutoBuilder.pathfindToPose(target, new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), 0);
    }

    // gets distance from speaker in meters
    public double getDistanceFromSpeaker() {
        Pose2d speakerPose = Constants.BLUE_SPEAKER_POSE;
        double x = getPose().getX() - speakerPose.getX();
        double y = getPose().getY() - speakerPose.getY();
        double distance = Math.sqrt(x * x + y * y);
        return distance;
    }

    public double getAngleToFaceSpeaker() {
        Pose2d speakerPose = Constants.BLUE_SPEAKER_POSE;
        double x = getPose().getX() - speakerPose.getX();
        double y = getPose().getY() - speakerPose.getY();
        double angle = Units.radiansToDegrees(Math.atan(y / x));
        return angle;
    }

    public double getRegToScoreInSpeaker() {
        double output = 56.9553 * Math.pow(0.8289, getDistanceFromSpeaker());
        return -output;
    }

    public double getTrigToScoreInSpeaker() {
        double height = Constants.HEIGHT_TO_SPEAKER_TARGET
                - (Constants.LENGTH_FROM_1ST_PIVOT_TO_2ND_PIVOT + Constants.HEIGHT_FROM_FLOOR_TO_1ST_PIVOT);
        double output = Units.radiansToDegrees(Math.atan(Units.inchesToMeters(height) / getDistanceFromSpeaker()));

        return -output;
    }

    public double getAverageofMethods() {
        return (getRegToScoreInSpeaker() + getTrigToScoreInSpeaker()) / 2;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModulePositions());

        final Pose2d estimatedPose = m_limeLight
                .getPose2DFromAlliance();
        if (m_limeLight.getFid() != -1) {
            poseEstimator.addVisionMeasurement(estimatedPose, m_limeLight.getTimeStamp());
        }

        for (SwerveModule mod : m_swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder",
                    mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated",
                    mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity",
                    mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("real robot pose x", getPose().getX());
        SmartDashboard.putNumber("real robot pose y", getPose().getY());
        SmartDashboard.putNumber("real robot pose rot", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Rot", getYaw().getDegrees());
        SmartDashboard.putNumber("Distance to Speaker", getDistanceFromSpeaker());
        SmartDashboard.putNumber("angle To Face Speaker", getAngleToFaceSpeaker());
        SmartDashboard.putNumber("angle to shoot in speaker", getTrigToScoreInSpeaker());

    }
}