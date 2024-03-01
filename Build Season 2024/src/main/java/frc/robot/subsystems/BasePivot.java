package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BasePivot extends SubsystemBase {
    private TalonFX bottomMotor;
    private TalonFX topMotor;

    private DutyCycleEncoder basePivotEncoder;

    public BasePivot() {
        bottomMotor = new TalonFX(Constants.BASE1_PIVOT_ID);
        topMotor = new TalonFX(Constants.BASE2_PIVOT_ID);

        basePivotEncoder = new DutyCycleEncoder(3);

        bottomMotor.setNeutralMode(NeutralModeValue.Brake);
        topMotor.setNeutralMode(NeutralModeValue.Brake);

        bottomMotor.setInverted(false);
        topMotor.setInverted(false);

        basePivotEncoder.setDistancePerRotation(360);

        basePivotEncoder.setPositionOffset(.791);

    }

    public void setPivot(double set) {
        bottomMotor.set(set);
        topMotor.set(set);
    }

    public double getPivotAngle() {
        return basePivotEncoder.getDistance() * -1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("bottomPivotAngle", getPivotAngle());
    }

}
