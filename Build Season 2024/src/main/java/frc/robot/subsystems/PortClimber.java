package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PortClimber extends SubsystemBase {

    private CANSparkMax portClimb;
    private boolean reverseClimb;

    public PortClimber() {
        portClimb = new CANSparkMax(22, MotorType.kBrushless);
        portClimb.setInverted(true);
        portClimb.setIdleMode(IdleMode.kBrake);
        reverseClimb = false;

    }

    public void setPortClimb(double set) {
        if (reverseClimb) {
            portClimb.set(set);
        } else {
            portClimb.set(-set);
        }
    }

    public void toggleClimbDirection() {
        if (reverseClimb) {
            reverseClimb = false;
        } else {
            reverseClimb = true;
        }
    }

}
