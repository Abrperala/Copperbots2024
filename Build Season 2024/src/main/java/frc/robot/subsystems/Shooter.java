package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public CANSparkMax leftShooter;
    public CANSparkMax rightShooter;
    public CANSparkMax intake;

    public Shooter() {
        // leftShooter = new CANSparkMax(Constants.SHOOT1_ID, MotorType.kBrushless);
        // rightShooter = new CANSparkMax(Constants.SHOOT2_ID, MotorType.kBrushless);
        // intake = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushed);

    }

    public void shoot() {
        // rightShooter.set(0);
        // leftShooter.set(-1);
    }

}
