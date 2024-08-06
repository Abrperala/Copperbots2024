package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTables;

public class Shooter extends SubsystemBase {
    private TalonFX shootermotor1Fx, shootermotor2Fx;
    private VelocityDutyCycle shooterVelocity = new VelocityDutyCycle(0);
    private ShooterState m_shooterState;
    private TalonFXConfiguration shooterConfigs;

    public Shooter() {
        shootermotor1Fx = new TalonFX(13);
        shootermotor2Fx = new TalonFX(14);
        m_shooterState = ShooterState.Standby;
    };

    public void configureShooterMotors() {
        shooterConfigs = new TalonFXConfiguration();

        shooterConfigs.Slot0.kS = 0.05;
        shooterConfigs.Slot0.kV = 0.12;
        shooterConfigs.Slot0.kP = 0.11;
        shooterConfigs.Slot0.kI = 0.0;
        shooterConfigs.Slot0.kD = 0.0;
        shooterConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 2;
        shooterConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .75;

        shootermotor1Fx.getConfigurator().apply(shooterConfigs);
        shootermotor2Fx.getConfigurator().apply(shooterConfigs);
        shootermotor1Fx.setNeutralMode(NeutralModeValue.Coast);
        shootermotor2Fx.setNeutralMode(NeutralModeValue.Coast);
        shootermotor1Fx.setInverted(false);
        shootermotor2Fx.setInverted(true);
    }

    /**
     * Sets the state of the shooter, used for spinning shooter wheels
     * 
     * @param shooterState
     */
    public void setShooterState(ShooterState shooterState) {
        m_shooterState = shooterState;
        shooterVelocity.Velocity = m_shooterState.shooterVelocity;
        if (m_shooterState == ShooterState.Standby) {
            stopShooters();
        } else {
            shootermotor1Fx.setControl(shooterVelocity);
            shootermotor2Fx.setControl(shooterVelocity);
        }
    };

    /**
     * returns the current state of the shooter
     * 
     * @return ShooterState
     */
    public ShooterState getShooterState() {
        return m_shooterState;
    }

    /**
     * stops the shooter motors
     * 
     */
    public void stopShooters() {
        shootermotor1Fx.stopMotor();
        shootermotor2Fx.stopMotor();
    }

    public enum ShooterState {
        Shooting(7000),
        Amping(6000),
        Reving(1000),
        Standby(0);

        public double shooterVelocity;

        private ShooterState(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }
    }

    @Override
    public void periodic() {
        NetworkTables.updateState("Shooter", getShooterState().toString());

    }
}
