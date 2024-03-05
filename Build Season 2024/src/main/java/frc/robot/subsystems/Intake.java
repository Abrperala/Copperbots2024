package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public DigitalInput beamBreak;
    public CANSparkMax intake;

    public Intake() {
        beamBreak = new DigitalInput(2);
        intake = new CANSparkMax(15, MotorType.kBrushed);

        intake.setIdleMode(IdleMode.kBrake);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public boolean isNotePresent() {
        return beamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("is note present", isNotePresent());

    }
}
