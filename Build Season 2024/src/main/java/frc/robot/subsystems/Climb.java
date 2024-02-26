package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    private CANSparkMax starboardCclimb;
    private CANSparkMax portClimb;

    public Climb() {
        starboardCclimb = new CANSparkMax(21, MotorType.kBrushless);
        portClimb = new CANSparkMax(22, MotorType.kBrushless);

        starboardCclimb.setInverted(false);
        portClimb.setInverted(true);
        starboardCclimb.setIdleMode(IdleMode.kBrake);
        portClimb.setIdleMode(IdleMode.kBrake);

    }

    public void setClimb(double set) {
        starboardCclimb.set(set);
        // portClimb.set(set);
    }

}
