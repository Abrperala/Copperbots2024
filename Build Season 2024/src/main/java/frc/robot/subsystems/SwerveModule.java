package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.lang.Math;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
    public int m_moduleNumber;
    public Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANcoder m_angleEncoder;

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.DRIVE_S, Constants.DRIVE_V,
            Constants.DRIVE_A);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0).withEnableFOC(false);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withEnableFOC(false);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        m_moduleNumber = moduleNumber;
        m_angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        m_angleEncoder = new CANcoder(moduleConstants.cancoderID, "DriveBus");
        configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID, "DriveBus");
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, "DriveBus");
        configDriveMotor();

        m_lastAngle = getState().angle;
    }

    /**
     * 
     * Set the desired SwerveModuleState of the module
     * 
     * @param desiredState desired SwerveModuleState
     * @param isOpenLoop   is the robot driving open loop
     * 
     */

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speeds of the motors based on the desired state.
     * 
     * @param desiredState The desired state of the swerve module.
     * @param isOpenLoop   Specifies whether the robot is driving open loop.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.MAX_SPEED;
            m_driveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToTalon(desiredState.speedMetersPerSecond,
                    Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            m_driveMotor.setControl(driveVelocity);
        }
    }

    /**
     * Sets the angle of the module based on the desired state.
     * 
     * @param desiredState Desired SwerveModuleState of the module.
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_SPEED * 0.01)) ? m_lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        anglePosition.Position = Conversions.degreesToTalon(angle.getDegrees(), Constants.ANGLE_GEAR_RATIO);
        m_angleMotor.setControl(anglePosition);
        m_lastAngle = angle;
    }

    /**
     * Retrieves the current angle of the module.
     * 
     * @return The current angle of the module.
     */
    private Rotation2d getAngle() {

        return Rotation2d.fromDegrees(
                Conversions.talonToDegrees(m_angleMotor.getPosition().getValue(), Constants.ANGLE_GEAR_RATIO));
    }

    /**
     * Retrieves the absolute angle from the CANCoder.
     * 
     * @return The absolute angle from the CANCoder.
     */
    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
    }

    public double getCANcoderWithOffset() {
        return m_angleEncoder.getAbsolutePosition().getValue() - m_angleOffset.getDegrees();
    }

    private Rotation2d waitForCANcoder() {
        /* wait for up to 250ms for a new CANcoder position */
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().waitForUpdate(250).getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToTalon(
                waitForCANcoder().getDegrees() - m_angleOffset.getDegrees(), Constants.ANGLE_GEAR_RATIO);
        m_angleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        m_angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        m_angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        m_driveMotor.getConfigurator().setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.talonToMPS(m_driveMotor.getVelocity().getValue(), Constants.WHEEL_CIRCUMFERENCE,
                        Constants.DRIVE_GEAR_RATIO),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.talonToMeters(m_driveMotor.getPosition().getValue(), Constants.WHEEL_CIRCUMFERENCE,
                        Constants.DRIVE_GEAR_RATIO),
                getAngle());
    }
}