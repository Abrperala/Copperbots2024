package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StarboardClimber extends SubsystemBase {
    private CANSparkMax starboardCclimb;
    private boolean reverseClimb;

    public StarboardClimber() {
        starboardCclimb = new CANSparkMax(21, MotorType.kBrushless);
        starboardCclimb.setInverted(false);
        starboardCclimb.setIdleMode(IdleMode.kBrake);
        reverseClimb = false;
    }

    public void setStarboardClimb(double set) {
        if (reverseClimb) {
            starboardCclimb.set(-set);
        } else {

            starboardCclimb.set(set);
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
