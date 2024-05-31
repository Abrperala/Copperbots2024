package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTables;
import frc.robot.Subsystems.Intake.IntakeState;

public class Arm extends SubsystemBase {

    private TalonFX leftarmmotorFX;
    private TalonFX rightarmmotorFX;
    private DutyCycleEncoder armEncoder;
    private ArmState armState;

    public Arm() {
        leftarmmotorFX = new TalonFX(16);
        rightarmmotorFX = new TalonFX(17);
        armEncoder = new DutyCycleEncoder(3);
        armState = ArmState.Standby;

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
        double rawAngle = armEncoder.getDistance() * -1;
        double wrappedAngle = (rawAngle % 360 + 360) % 360;
        if (wrappedAngle > 180) {
            wrappedAngle -= 360;
        }
        return wrappedAngle;
    }

    /**
     * TalonFX method to turn arm motor clockwise at 1 volt
     *
     * @Check for direction of motor
     */
    public void turnClockwise() {
        leftarmmotorFX.setVoltage(1);
        rightarmmotorFX.setVoltage(1);
    }

    /**
     * TalonFX method to turn arm motor counterclockwise at 1 volt
     *
     * @Check for direction of motor
     */
    public void turnCounterClockwise() {
        leftarmmotorFX.setVoltage(-1);
        rightarmmotorFX.setVoltage(-1);
    }

    /**
     * TalonFX method using stopMotor() to turn off motor
     *
     * 
     */
    public void stopTurning() {
        leftarmmotorFX.stopMotor();
        rightarmmotorFX.stopMotor();

    }
    public void setArmState(ArmState state){
        armState = state;
        
    }

    public ArmState getArmState(){
        return armState;
    }

    public enum ArmState {
        Standby(90),
        Intaking(-10),
        Sourcing(63);

        public double angle;

        private ArmState(double angle) {
            this.angle = angle;
        };
    }

      @Override
    public void periodic() {
        NetworkTables.updateState("Arm", getArmState().toString());
    }

}