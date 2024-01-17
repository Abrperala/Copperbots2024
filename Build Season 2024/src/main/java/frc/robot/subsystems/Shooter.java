package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public CANSparkMax leftShooter;
    public CANSparkMax rightShooter;
    public CANSparkMax intake;

    public Shooter() {
        leftShooter = new CANSparkMax(13, MotorType.kBrushless);
        rightShooter = new CANSparkMax(14, MotorType.kBrushless);
        // intake = new CANSparkMax(15, MotorType.kBrushed);

    }

    public void shoot() {
        rightShooter.set(0);
        leftShooter.set(-1);
    }

}
