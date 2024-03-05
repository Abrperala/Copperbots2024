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
        if (isinRange()) {
            bottomMotor.set(set);
            topMotor.set(set);
        } else {
            System.out.println("Base Pivot Bad");
        }
    }

    public void manualSetPivot(double set) {
        bottomMotor.set(set);
        topMotor.set(set);
    }

    /**
     * Gets the wrapped pivot angle within the range of (-180, 180] degrees.
     * 
     * This method retrieves the raw pivot angle from the DutyCycleEncoder, inverses
     * the encoding direction, and ensures
     * that the angle is wrapped within the specified range. The wrapping prevents
     * jumps of plus or minus 360 degrees, providing a consistent angle.
     *
     * @return The wrapped pivot angle in degrees within the range (-180, 180].
     */
    public double getPivotAngle() {
        double rawAngle = basePivotEncoder.getDistance() * -1;
        double wrappedAngle = (rawAngle % 360 + 360) % 360;
        if (wrappedAngle > 180) {
            wrappedAngle -= 360;
        }
        return wrappedAngle;
    }

    public boolean isinRange() {
        if (getPivotAngle() > Constants.MAX_BOTTOM_PIVOT_ANGLE || getPivotAngle() < Constants.MIN_BOTTOM_PIVOT_ANGLE) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Bounded Bottom Pivot Angle", getPivotAngle());
        SmartDashboard.putNumber("real Bottom Pivot Angle", basePivotEncoder.getDistance());
    }

}
