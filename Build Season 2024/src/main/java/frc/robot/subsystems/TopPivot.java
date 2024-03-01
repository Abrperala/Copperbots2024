package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.GeneralUtils;
import frc.robot.Constants;

public class TopPivot extends SubsystemBase {
    // private CANSparkMax top1;
    private TalonFX topMotor;
    private DutyCycleEncoder topPivotEncoder;

    public TopPivot() {
        topMotor = new TalonFX(Constants.TOP_PIVOT_ID);
        // top1 = new CANSparkMax(Constants.TOP_PIVOT_ID, MotorType.kBrushless);
        topPivotEncoder = new DutyCycleEncoder(1);

        topMotor.setNeutralMode(NeutralModeValue.Brake);
        topMotor.setInverted(false);
        // top1.setIdleMode(IdleMode.kBrake);
        // top1.setInverted(false);

        topPivotEncoder.setDistancePerRotation(360);
        // set the position offset to about 10 degrees
        topPivotEncoder.setPositionOffset(0.0187);

    }

    public void setPivot(double set) {
        topMotor.set(set);
        // top1.set(set);
    }

    public void stopPivot() {
        topMotor.set(0);
        // top1.set(0);
    }

    public double getPivotAngle() {
        return topPivotEncoder.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("topPivotAngle", getPivotAngle());
    }
}
