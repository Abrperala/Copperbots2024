package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TopPivot extends SubsystemBase {
    private TalonFX topMotor;
    private DutyCycleEncoder topPivotEncoder;

    public TopPivot() {
        topMotor = new TalonFX(Constants.TOP_PIVOT_ID);
        topPivotEncoder = new DutyCycleEncoder(1);

        topMotor.setNeutralMode(NeutralModeValue.Brake);
        topMotor.setInverted(false);

        topPivotEncoder.setDistancePerRotation(360);
        topPivotEncoder.setPositionOffset(0.0187);

    }

    public void setPivot(double set) {
        if (isinRange()) {
            topMotor.set(set);
        } else {
            System.out.println("Top Pivot Bad");
        }

    }

    public void manualSetPivot(double set) {
        topMotor.set(set);
    }

    public void stopPivot() {
        topMotor.set(0);
    }

    /**
     * Gets the wrapped pivot angle within the range of (-180, 180] degrees.
     * 
     * This method retrieves the raw pivot angle from the DutyCycleEncoder and
     * ensures
     * that the angle is wrapped within the specified range. The wrapping prevents
     * jumps of plus or minus 360 degrees, providing a consistent angle.
     *
     * @return The wrapped pivot angle in degrees within the range (-180, 180].
     */
    public double getPivotAngle() {
        double rawAngle = topPivotEncoder.getDistance();
        double wrappedAngle = (rawAngle % 360 + 360) % 360;
        if (wrappedAngle > 180) {
            wrappedAngle -= 360;
        }
        return wrappedAngle;
    }

    public boolean isinRange() {
        if (getPivotAngle() > Constants.MAX_TOP_PIVOT_ANGLE || getPivotAngle() < Constants.MIN_TOP_PIVOT_ANGLE) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Bounded Top Pivot Angle", getPivotAngle());
        SmartDashboard.putNumber("Real Top Pivot Angle", topPivotEncoder.getDistance());

    }
}
