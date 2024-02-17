package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BasePivot extends SubsystemBase {
    private CANSparkMax base1;
    private CANSparkMax base2;

    private Encoder basePivotEncoder;

    public BasePivot() {
        base1 = new CANSparkMax(Constants.BASE1_PIVOT_ID, MotorType.kBrushless);
        base2 = new CANSparkMax(Constants.BASE2_PIVOT_ID, MotorType.kBrushless);

        // A is blue, B is yellow
        basePivotEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);

        base1.setIdleMode(IdleMode.kBrake);
        base2.setIdleMode(IdleMode.kBrake);

        base1.setInverted(false);
        base2.setInverted(false);

        basePivotEncoder.setDistancePerPulse(360 / 2048.0 / 3);

    }

    public void setPivot(double set) {
        base1.set(set);
        base2.set(set);
    }

    public double getPivotAngle() {
        return basePivotEncoder.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("bottomPivotAngle", getPivotAngle());
    }

}
