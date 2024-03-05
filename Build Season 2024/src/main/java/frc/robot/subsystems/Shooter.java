package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
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

    public Shooter() {
        topShooter = new TalonFX(Constants.SHOOT1_ID);
        bottomShooter = new TalonFX(Constants.SHOOT2_ID);

        m_request = new VelocityVoltage(0).withSlot(0);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        topShooter.getConfigurator().apply(slot0Configs);
        bottomShooter.getConfigurator().apply(slot0Configs);

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
