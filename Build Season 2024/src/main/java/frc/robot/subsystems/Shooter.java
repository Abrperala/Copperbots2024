package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public CANSparkMax topShooter;
    public CANSparkMax bottomShooter;

    private RelativeEncoder m_topEncoder;
    private RelativeEncoder m_bottomEncoder;

    public Shooter() {
        topShooter = new CANSparkMax(Constants.SHOOT1_ID, MotorType.kBrushless);
        bottomShooter = new CANSparkMax(Constants.SHOOT2_ID, MotorType.kBrushless);

        topShooter.setInverted(false);
        bottomShooter.setInverted(true);
        topShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setOpenLoopRampRate(1);
        topShooter.setOpenLoopRampRate(1);

        m_topEncoder = topShooter.getEncoder();
        m_bottomEncoder = bottomShooter.getEncoder();

        m_topEncoder.setVelocityConversionFactor(Constants.SHOOTER_GEARING);
        m_bottomEncoder.setVelocityConversionFactor(Constants.SHOOTER_GEARING);
    }

    public void setTopShooterSpeed(double speed) {
        topShooter.set(speed);

    }

    public void setBottomShooterSpeed(double speed) {
        bottomShooter.set(speed);
    }

    public void setTopShooterVoltage(double volts) {
        topShooter.setVoltage(volts);

    }

    public void setBottomShooterVoltage(double volts) {
        bottomShooter.setVoltage(volts);
    }

    public double getTopEncoderVelocity() {
        return m_topEncoder.getVelocity();
    }

    public double getBottomEncoderVelocity() {
        return m_bottomEncoder.getVelocity();
    }

    public boolean shooterAtSpeed() {
        return getTopEncoderVelocity() > Constants.SHOOTER_TARGET_RPM - 200
                && getBottomEncoderVelocity() > Constants.SHOOTER_TARGET_RPM - 200;
    }

    public double getShooterSpeed() {
        return topShooter.get();
    }

    public boolean shooterNotRunning() {
        return topShooter.get() == 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter Ready?", shooterAtSpeed());
        SmartDashboard.putNumber("Top Shooter RPM", getTopEncoderVelocity());
        SmartDashboard.putNumber("Bottom Shooter RPM", getBottomEncoderVelocity());
        SmartDashboard.putBoolean("Shooter not running?", shooterNotRunning());
        SmartDashboard.putNumber("shooter setSpeed", getShooterSpeed());
    }

}
