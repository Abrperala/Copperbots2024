package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTables;

public class Arm extends SubsystemBase {

    private TalonFX leftarmmotorFX;
    private TalonFX rightarmmotorFX;
    private DutyCycleEncoder armEncoder;
    private ArmState armState;
    private ProfiledPIDController armPID;
    private Constraints armConstraints;

    public Arm() {
        leftarmmotorFX = new TalonFX(16);
        rightarmmotorFX = new TalonFX(17);
        armEncoder = new DutyCycleEncoder(3);

        armConstraints = new Constraints(3000, 1200);
        armPID = new ProfiledPIDController(.013, 0, 0, armConstraints);
        armEncoder.setDistancePerRotation(360);
        armEncoder.setPositionOffset(.791);

        rightarmmotorFX.getConfigurator().apply(new TalonFXConfiguration());
        leftarmmotorFX.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();

        fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightarmmotorFX.getConfigurator().apply(fx_cfg);
        leftarmmotorFX.getConfigurator().apply(fx_cfg);

        rightarmmotorFX.setInverted(false);
        leftarmmotorFX.setInverted(false);

        armState = ArmState.Standby;

    }

    public void setArmPosition() {
        double output = armPID.calculate(getArmAngle(), getArmState().angle);
        leftarmmotorFX.set(output);
        rightarmmotorFX.set(output);

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
    public double getArmAngle() {
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

    public void setArmState(ArmState state) {
        armState = state;
        armPID.reset(getArmAngle());

    }

    public ArmState getArmState() {
        return armState;
    }

    public enum ArmState {
        Standby(90),
        Intaking(-10),
        Sourcing(63),
        Amping(119);

        public double angle;

        private ArmState(double angle) {
            this.angle = angle;
        };
    }

    @Override
    public void periodic() {
        setArmPosition();
        NetworkTables.updateState("Arm", getArmState().toString());
        SmartDashboard.putNumber("Arm Encoder", getArmAngle());
    }

}