package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
     * 
     * Set the speeds of the motors
     * 
     * @param desiredState desired SwerveModuleState of the module
     * @param isOpenLoop is the robot driving open loop
     * 
     */

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.MAX_SPEED;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    /**
     * 
     * Set the angle of the module
     * 
     * @param desiredState desired SwerveModuleState of the module 
     * 
     */

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_SPEED * 0.01)) ? m_lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.AZIMUTH_GEAR_RATIO));
        m_lastAngle = angle;
    }

    /**
     * 
     * @return angle of the module
     * 
     */

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.AZIMUTH_GEAR_RATIO));
    }

    /**
     * 
     * @return CANCoder angle
     * 
     */

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }
    
    public Rotation2d getAngleOffset(){
        return m_angleOffset;
    }
    
    /**
     * 
     * Reset the module to the absolute position
     * 
     */

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset.getDegrees(), Constants.AZIMUTH_GEAR_RATIO);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    
    /**
     * 
     * Run the azimuth motor config
     * 
     */

    private void configAngleEncoder(){        
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);
        m_angleEncoder.setPositionToAbsolute();
    }

    /**
     * 
     * Configure 
     * 
     */

    private void configAngleMotor(){
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(CTREConfigs.swerveAngleFXConfig);
        m_angleMotor.setInverted(Constants.FRONT_LEFT_AZIMUTH_REVERSED);
        m_angleMotor.setNeutralMode(Constants.AZIMUTH_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig);
        m_driveMotor.setInverted(Constants.FRONT_LEFT_DRIVE_REVERSED);
        m_driveMotor.setNeutralMode(Constants.DRIVE_NEUTRAL_MODE);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }
}