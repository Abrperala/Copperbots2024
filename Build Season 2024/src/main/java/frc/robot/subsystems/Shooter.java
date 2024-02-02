package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public CANSparkMax topShooter;
    public CANSparkMax bottomShooter;

    private RelativeEncoder m_topEncoder;
    private RelativeEncoder m_bottomEncoder;

    public Shooter() {
        topShooter = new CANSparkMax(13, MotorType.kBrushless);
        bottomShooter = new CANSparkMax(14, MotorType.kBrushless);

        m_topEncoder = topShooter.getEncoder();
        m_bottomEncoder = bottomShooter.getEncoder();
    }

    public void setShooterSpeed(double speed) {
        topShooter.set(speed);
        bottomShooter.set(-speed);
    }

    public double getTopEncoderVelocity() {
        return m_topEncoder.getVelocity() * Constants.SHOOTER_GEARING;
    }

    public double getBottomEncoderVelocity() {
        return m_bottomEncoder.getVelocity() * Constants.SHOOTER_GEARING;
    }

    public boolean shooterAtSpeed() {
        return getTopEncoderVelocity() > Constants.SHOOTER_TARGET_RPM
                && getBottomEncoderVelocity() > Constants.SHOOTER_TARGET_RPM;
    }

}
