package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public final class CTREConfigs {

    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();


    public CTREConfigs(){

        CurrentLimitsConfigs angleLimitConfig = new CurrentLimitsConfigs();

        angleLimitConfig.SupplyCurrentLimit = Constants.AZIMUTH_CONTINUOUS_CURRENT_LIMIT;
        angleLimitConfig.SupplyCurrentLimitEnable = Constants.AZIMUTH_ENABLE_CURRENT_LIMIT;
        angleLimitConfig.SupplyCurrentThreshold = Constants.AZIMUTH_PEAK_CURRENT_LIMIT;
        angleLimitConfig.SupplyTimeThreshold = Constants.AZIMUTH_PEAK_CURRENT_DURATION;

        swerveAngleFXConfig.Slot0.kP = Constants.AZIMUTH_P;
        swerveAngleFXConfig.Slot0.kI = Constants.AZIMUTH_I;
        swerveAngleFXConfig.Slot0.kD = Constants.AZIMUTH_D;
        swerveAngleFXConfig.Slot0.kS = Constants.AZIMUTH_F;
        swerveAngleFXConfig.CurrentLimits = angleLimitConfig;


       CurrentLimitsConfigs driveLimitConfig = new CurrentLimitsConfigs();

       driveLimitConfig.SupplyCurrentLimit = Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
       driveLimitConfig.SupplyCurrentLimitEnable = Constants.DRIVE_ENABLE_CURRENT_LIMIT;
       driveLimitConfig.SupplyCurrentThreshold = Constants.DRIVE_PEAK_CURRENT_LIMIT;
       driveLimitConfig.SupplyTimeThreshold = Constants.DRIVE_PEAK_CURRENT_DURATION;


       OpenLoopRampsConfigs driveOpenLoopRampsConfigs = new OpenLoopRampsConfigs();
      
       driveOpenLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.OPEN_LOOP_RAMP;


       ClosedLoopRampsConfigs driveClosedLoopRampsConfigs = new ClosedLoopRampsConfigs();

       driveClosedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.CLOSED_LOOP_RAMP;


        swerveDriveFXConfig.Slot0.kP = Constants.DRIVE_P;
        swerveDriveFXConfig.Slot0.kI = Constants.DRIVE_I;
        swerveDriveFXConfig.Slot0.kD = Constants.DRIVE_D;
        swerveDriveFXConfig.Slot0.kS = Constants.DRIVE_F;        
        swerveDriveFXConfig.CurrentLimits = driveLimitConfig;
        swerveDriveFXConfig.OpenLoopRamps = driveOpenLoopRampsConfigs;
        swerveDriveFXConfig.ClosedLoopRamps = driveClosedLoopRampsConfigs;
        
        /* Swerve CANCoder Configuration */
        
        //swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition; is Needed?
        //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond; is Needed

        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.FRONT_LEFT_CANCODER_REVERSED;
    

      
    }
}