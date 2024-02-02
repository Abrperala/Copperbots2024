package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public CANSparkMax intake;

    public Intake() {
        intake = new CANSparkMax(15, MotorType.kBrushed);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }
}
