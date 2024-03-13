package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public TalonFX topShooter;
    public TalonFX bottomShooter;
    public VelocityVoltage m_request;
    private VelocityDutyCycle shooterVelocity = new VelocityDutyCycle(0);
    private TalonFXConfiguration shooterConfigs;

    public Shooter() {
        topShooter = new TalonFX(Constants.SHOOT1_ID);
        bottomShooter = new TalonFX(Constants.SHOOT2_ID);
        shooterConfigs = new TalonFXConfiguration();

        m_request = new VelocityVoltage(0).withSlot(0);

        shooterConfigs.Slot0.kS = 0.05;
        shooterConfigs.Slot0.kV = 0.12;
        shooterConfigs.Slot0.kP = 0.11;
        shooterConfigs.Slot0.kI = 0.0;
        shooterConfigs.Slot0.kD = 0.0;
        shooterConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 2;
        shooterConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;

        topShooter.getConfigurator().apply(shooterConfigs);
        bottomShooter.getConfigurator().apply(shooterConfigs);
        topShooter.setInverted(false);
        bottomShooter.setInverted(true);
        topShooter.setNeutralMode(NeutralModeValue.Coast);
        bottomShooter.setNeutralMode(NeutralModeValue.Coast);

    }

    public void shooterRun(double velocity) {
        shooterVelocity.Velocity = velocity;
        topShooter.setControl(shooterVelocity);
        bottomShooter.setControl(shooterVelocity);

    }

    public double getTopEncoderVelocity() {
        return topShooter.getVelocity().getValueAsDouble() * Constants.SHOOTER_GEARING * 60;
    }

    public double getBottomEncoderVelocity() {
        return bottomShooter.getVelocity().getValueAsDouble() * Constants.SHOOTER_GEARING * 60;

    }

    public void stopShooter() {
        topShooter.stopMotor();
        bottomShooter.stopMotor();
    }

    public boolean shooterAtSpeed() {
        return getTopEncoderVelocity() > Constants.SHOOTER_TARGET_RPM
                && getBottomEncoderVelocity() > Constants.SHOOTER_TARGET_RPM;

    }

    public double getShooterSpeed() {
        return topShooter.get();
    }

    public boolean shooterNotRunning() {
        return topShooter.get() == 0;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Shooter Ready?", shooterAtSpeed());
        SmartDashboard.putNumber("Top Shooter RPM", getTopEncoderVelocity());
        SmartDashboard.putNumber("Bottom Shooter RPM", getBottomEncoderVelocity());
        SmartDashboard.putBoolean("Shooter not running?", shooterNotRunning());
        SmartDashboard.putNumber("shooter setSpeed", getShooterSpeed());
    }

}
