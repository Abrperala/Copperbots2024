package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveMods;
    public AHRS m_gyro;

    public SwerveDrivetrain() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        m_gyro.calibrate();
        zeroGyro();

        m_swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Mod0.constants),
            new SwerveModule(1, Constants.Mod1.constants),
            new SwerveModule(2, Constants.Mod2.constants),
            new SwerveModule(3, Constants.Mod3.constants)
        };

        m_swerveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getModulePositions());

        setOdometryForOdometryAlign();

    }

        /**
         * Drive method for swerve drivetrain.
         * 
         * @param translation       The desired translation vector.
         * @param rotation          The desired rotation value.
         * @param fieldRelative     Whether the translation is field relative.
         * @param isOpenLoop        Whether the drive is open loop.
         */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            m_gyro.getRotation2d()
        )
        : new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }  
    
    
    /**
     * Set chassis speeds for the swerve drivetrain.
     * 
     * @param targetSpeeds  The target chassis speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(Constants.swerveKinematics.toSwerveModuleStates(targetSpeeds));
    }

    /**
     * Set desired states for each swerve module.
     * 
     * @param desiredStates  The desired states for each swerve module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED);
        
        for(SwerveModule mod : m_swerveMods){
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
     * @param pose  The desired pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

 
    public void setOdometryToOffset() {
        m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(0.0), getModulePositions(), new Pose2d(-6.14, 1.21, Rotation2d.fromDegrees(0.0)));
    }

    public void setOdometryForOdometryAlign() {
        m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(0.0), getModulePositions(), new Pose2d(13.56, 5.2, Rotation2d.fromDegrees(0.0)));
    }

    
    /**
     * Gets the current state of each swerve module.
     * 
     * @return Array of SwerveModuleState representing the current state of each swerve module.
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveMods){
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Gets the current position of each swerve module.
     * 
     * @return Array of SwerveModulePosition representing the current position of each swerve module.
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveMods){
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
    public void zeroGyro(){
        m_gyro.reset();
    }

    /**
     * Gets the current yaw (rotation around Z-axis) from the gyro.
     * 
     * @return The current yaw as a Rotation3d.
     */
    public Rotation2d getYaw() {
        return m_gyro.getRotation2d();
         
    }

    @Override
    public void periodic(){
        m_swerveOdometry.update(getYaw(), getModulePositions());  
    
        for(SwerveModule mod : m_swerveMods){
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + "Absolute Angle", mod.getCanCoder().getDegrees() - mod.getAngleOffset().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("real robot pose x", getPose().getX());
        SmartDashboard.putNumber("real robot pose y", getPose().getY());
        SmartDashboard.putNumber("real robot pose rot", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro", m_gyro.getYaw() + 180);
    }
}