package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivots extends SubsystemBase {

    CANSparkMax base1;
    CANSparkMax base2;
    CANSparkMax tall1;

    Encoder bottomPivotEncoder;
    Encoder topPivotEncoder;

    public Pivots() {
        // base1 = new CANSparkMax(16, MotorType.kBrushless);
        // base2 = new CANSparkMax(Constants.BASE2_PIVOT_ID, MotorType.kBrushless);
        // tall1 = new CANSparkMax(18, MotorType.kBrushless);

        // bottomPivotEncoder = new Encoder(5, 4, false, CounterBase.EncodingType.k4X);
        // topPivotEncoder = new Encoder(9, 8, false, CounterBase.EncodingType.k4X);

        // base1.setIdleMode(IdleMode.kBrake);
        // base2.setIdleMode(IdleMode.kBrake);
        // tall1.setIdleMode(IdleMode.kBrake);
        // base1.setInverted(false);
        // base2.setInverted(false);
        // tall1.setInverted(false);

        // bottomPivotEncoder.setDistancePerPulse(360 / 2048.0);
        // topPivotEncoder.setDistancePerPulse(360 / 2048.0);
    }

    public void ChangeBasePivot(double set) {

        // base1.set(set);
        // base2.set(set);
    }

    public void ChangeTopPivot(double set) {
        // tall1.set(set);

    }

    // public double getTopPivotAngle() {
    // return topPivotEncoder.getDistance();
    // }

    // public double getBottomPivotAngle() {
    // return bottomPivotEncoder.getDistance();
    // }

    public void zeroTopEncoder() {
        // topPivotEncoder.reset();
    }

    public void zeroBottomEncoder() {
        // bottomPivotEncoder.reset();
    }

    public double getShotAngle(double firstPivotAngle, double secondPivotAngle) {
        return secondPivotAngle - firstPivotAngle + 90;
    }

    /*
     * Based on a coordinate system where the first angle
     * (the first pivot) has its reference at the left horizontal (is zeroed) and
     * increases in the clockwise direction. The second angle (the second pivot) has
     * its reference at the right horizontal (is zeroed) while the first pivot is
     * 90, and
     * increases in the
     * counterclockwise direction.
     */

    /**
     * Calculates the height of the second pivot based on the angle (in Degrees) of
     * the first
     * pivot.
     *
     * @param firstPivotAngle Angle of the first pivot.
     * @return Height of the second pivot.
     */
    public static double get2ndPivotHeight(double firstPivotAngle) {
        return Constants.LENGTH_FROM_1ST_PIVOT_TO_2ND_PIVOT * Math.sin(Math.toRadians(firstPivotAngle));
    }

    /**
     * Calculates the height of the shooter exit point based on the angle (in
     * Degrees) of the
     * second pivot.
     *
     * @param secondPivotAngle Angle of the second pivot.
     * @return Height of the shooter exit point.
     */
    public static double getShooterExitHeight(double secondPivotAngle) {
        return Constants.LENGTH_FROM_2ND_PIVOT_TO_SHOOTER_EXIT * Math.sin(Math.toRadians(secondPivotAngle));
    }

    /**
     * Calculates the total height from the floor to the shooter exit point,
     * considering both pivot angles (in Degrees).
     *
     * @param firstPivotAngle  Angle of the first pivot.
     * @param secondPivotAngle Angle of the second pivot.
     * @return Total height from the floor to the shooter exit point.
     */
    public static double getHeightFromFloorToShooterExit(double firstPivotAngle, double secondPivotAngle) {
        return Constants.HEIGHT_FROM_FLOOR_TO_1ST_PIVOT + get2ndPivotHeight(firstPivotAngle)
                + getShooterExitHeight(secondPivotAngle - firstPivotAngle + 90);
    }

    /**
     * Calculates the base distance from the first pivot to the second pivot based
     * on the first pivot angle (in Degrees).
     *
     * @param firstPivotAngle Angle of the first pivot.
     * @return Base distance from the first pivot to the second pivot.
     */
    public static double getBaseFromPivotToPivot(double firstPivotAngle) {
        return Constants.LENGTH_FROM_1ST_PIVOT_TO_2ND_PIVOT * Math.cos(Math.toRadians(firstPivotAngle));
    }

    /**
     * Calculates the base distance from the second pivot to the shooter exit point
     * based on the second pivot angle (in Degrees).
     *
     * @param secondPivotAngle Angle of the second pivot.
     * @return Base distance from the second pivot to the shooter exit point.
     */
    public static double getBaseFrom2ndPivotToShooterExit(double secondPivotAngle) {
        return Constants.LENGTH_FROM_2ND_PIVOT_TO_SHOOTER_EXIT * Math.cos(Math.toRadians(secondPivotAngle));
    }

    /**
     * Calculates the offset of the shooter exit point to the robot, considering
     * both pivot angles (in Degrees).
     *
     * @param firstPivotAngle  Angle of the first pivot.
     * @param secondPivotAngle Angle of the second pivot.
     * @return Offset of the shooter exit point to the robot.
     */
    public static double getOffsetOfShooterExitToRobot(double firstPivotAngle, double secondPivotAngle) {
        return getBaseFromPivotToPivot(firstPivotAngle)
                + getBaseFrom2ndPivotToShooterExit(secondPivotAngle + firstPivotAngle + 90);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("bottomPivotAngle", getBottomPivotAngle());
        // SmartDashboard.putNumber("topPivotAngle", getTopPivotAngle());
        // SmartDashboard.putNumber("ShotAngle", getShotAngle(getBottomPivotAngle(),
        // getTopPivotAngle()));
        // SmartDashboard.putNumber("shooterExitHeight",
        // getHeightFromFloorToShooterExit(getTopPivotAngle(), getBottomPivotAngle()));
        // SmartDashboard.putNumber("shooterExitDistance",
        // getOffsetOfShooterExitToRobot(getTopPivotAngle(), getBottomPivotAngle()));
    }

}
