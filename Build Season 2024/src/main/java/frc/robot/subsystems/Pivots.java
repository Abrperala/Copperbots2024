package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivots extends SubsystemBase {

    CANSparkMax base1;
    CANSparkMax base2;
    CANSparkMax tall1;

    public Pivots() {
        // base1 = new CANSparkMax(Constants.BASE1_PIVOT_ID, MotorType.kBrushless);
        // base2 = new CANSparkMax(Constants.BASE2_PIVOT_ID, MotorType.kBrushless);
        // tall1 = new CANSparkMax(Constants.TALL_PIVOT_ID, MotorType.kBrushless);

        // base1.setIdleMode(IdleMode.kBrake);
        // base2.setIdleMode(IdleMode.kBrake);
        // tall1.setIdleMode(IdleMode.kBrake);
        // base1.setInverted(false);
        // base2.setInverted(false);
        // tall1.setInverted(false);

    }

    public void ChangeBasePivot(double set) {
        // base1.set(set);
        // base2.set(set);
    }

    public void ChangeTallPivot(double set) {
        // tall1.set(set);

    }
}
