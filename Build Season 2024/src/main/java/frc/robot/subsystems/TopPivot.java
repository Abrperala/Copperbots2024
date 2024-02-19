package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TopPivot extends SubsystemBase {
    private CANSparkMax top1;
    private DutyCycleEncoder topPivotEncoder;

    public TopPivot() {
        top1 = new CANSparkMax(Constants.TOP_PIVOT_ID, MotorType.kBrushless);
        topPivotEncoder = new DutyCycleEncoder(2);

        top1.setIdleMode(IdleMode.kBrake);
        top1.setInverted(false);

        topPivotEncoder.setDistancePerRotation(360);
        // set the position offset to about 40 degrees
        topPivotEncoder.setPositionOffset(.11);
    }

    public void setPivot(double set) {
        top1.set(set);
    }

    public void stopPivot() {
        top1.set(0);
    }

    public double getPivotAngle() {
        return topPivotEncoder.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("topPivotAngle", getPivotAngle());
    }
}
