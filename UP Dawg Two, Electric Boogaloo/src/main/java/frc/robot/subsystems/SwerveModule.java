package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.lang.Math;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int m_moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DRIVE_S, Constants.DRIVE_V, Constants.DRIVE_A);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        m_moduleNumber = moduleNumber;
        m_angleOffset = moduleConstants.angleOffset;
        /* Angle Encoder Config */
        m_angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        m_lastAngle = getState().angle;
    }


        
    /**
     * 
     * Set the desired SwerveModuleState of the module
     * 
     * @param desiredState desired SwerveModuleState
     * @param isOpenLoop is the robot driving open loop
     * 
     */

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speeds of the motors based on the desired state.
     * 
     * @param desiredState  The desired state of the swerve module.
     * @param isOpenLoop    Specifies whether the robot is driving open loop.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
             // Convert speed to percent output and set drive motor
            double percentOutput = desiredState.speedMetersPerSecond / Constants.MAX_SPEED;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
             // Convert velocity and apply feedforward for closed-loop control
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    /**
     * Sets the angle of the module based on the desired state.
     * 
     * @param desiredState  Desired SwerveModuleState of the module.
     */
    private void setAngle(SwerveModuleState desiredState){
        // Determine the angle to set based on the desired state and prevent jittering at low speeds
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_SPEED * 0.01)) ? m_lastAngle : desiredState.angle;
        
        // Convert and set the angle to the azimuth motor
        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.AZIMUTH_GEAR_RATIO));
        m_lastAngle = angle; // Update the last known angle
    }

    /**
     * Retrieves the current angle of the module.
     * 
     * @return The current angle of the module.
     */
    private Rotation2d getAngle(){
        // Retrieve the angle from the azimuth motor encoder
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.AZIMUTH_GEAR_RATIO));
    }

    /**
     * Retrieves the absolute angle from the CANCoder.
     * 
     * @return The absolute angle from the CANCoder.
     */
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }
    
    
    /**
     * Retrieves the angle offset of the module.
     * 
     * @return The angle offset of the module.
     */
    public Rotation2d getAngleOffset(){
        return m_angleOffset;
    }
 
    /**
     * Resets the module to the absolute position based on the CANCoder.
     */
    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset.getDegrees(), Constants.AZIMUTH_GEAR_RATIO);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * Configures settings for the azimuth motor encoder (CANCoder).
     */
    private void configAngleEncoder(){        
        // Reset configurations, apply specific settings from CTREConfigs, and set to absolute position
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);
        m_angleEncoder.setPositionToAbsolute();
    }

    /**
    * Configures settings for the azimuth motor.
    */
    private void configAngleMotor(){
        // Reset configurations, apply specific settings from CTREConfigs, set inversion, neutral mode, and reset to absolute position
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(CTREConfigs.swerveAngleFXConfig);
        m_angleMotor.setInverted(Constants.FRONT_LEFT_AZIMUTH_REVERSED);
        m_angleMotor.setNeutralMode(Constants.AZIMUTH_NEUTRAL_MODE);
        resetToAbsolute();
    }

    /**
    * Configures settings for the drive motor.
    */
    private void configDriveMotor(){      
        // Reset configurations, apply specific settings from CTREConfigs, set inversion, neutral mode, and reset position to 0  
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig);
        m_driveMotor.setInverted(Constants.FRONT_LEFT_DRIVE_REVERSED);
        m_driveMotor.setNeutralMode(Constants.DRIVE_NEUTRAL_MODE);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    
    /**
     * Retrieves the current state of the swerve module.
     * 
     * @return The current state of the swerve module.
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO), 
            getAngle()
        ); 
    }

    /**
     * Retrieves the current position of the swerve module.
     * 
     * @return The current position of the swerve module.
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }
}