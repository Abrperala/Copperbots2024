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

public class Wrist extends SubsystemBase {
    private TalonFX wristmotorFx;
    private WristState wristState;
    private DutyCycleEncoder wristEncoder;
    private ProfiledPIDController wristPID;
    private Constraints wristPIDConstraints;

    public Wrist() {
        wristmotorFx = new TalonFX(18);
        wristEncoder = new DutyCycleEncoder(1);
        wristPIDConstraints = new Constraints(3000, 1200);
        wristPID = new ProfiledPIDController(.012, 0, 0, wristPIDConstraints);
        wristEncoder.setDistancePerRotation(360);
        wristEncoder.setPositionOffset(0.0187);

        wristmotorFx.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
        fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristmotorFx.getConfigurator().apply(fx_cfg);

        wristState = WristState.Standby;

    }

    public void setWristPosition() {
        wristmotorFx.set(wristPID.calculate(getWristAngle(), getWristState().angle));

    }

    public double getWristEncoder() {
        return wristEncoder.get();
    }

    public double getWristAngle() {
        double rawAngle = wristEncoder.getDistance();
        double wrappedAngle = (rawAngle % 360 + 360) % 360;
        if (wrappedAngle > 180) {
            wrappedAngle -= 360;
        }
        return wrappedAngle;
    }

    /**
     * Uses the TalonFX method stopMotor() to stop the motor
     * 
     */
    public void stopWrist() {
        wristmotorFx.stopMotor();

    }

    public void setWristState(WristState state) {
        wristState = state;
        wristPID.reset(getWristAngle());
    }

    public WristState getWristState() {
        return wristState;
    }

    public enum WristState {
        Standby(0),
        Intaking(36),
        Sourcing(63),
        Shooting(0),
        SpeakerShot(-52),
        Amping(-5);

        public double angle;

        private WristState(double angle) {
            this.angle = angle;
        };
    }

    @Override
    public void periodic() {
        setWristPosition();

        NetworkTables.updateState("Wrist", getWristState().toString());
        SmartDashboard.putNumber("wrist encoder", getWristAngle());
        // SmartDashboard.putNumber("cancoder wrist", getCanCoder());
        // SmartDashboard.putNumber("cancoder id", wristCancoder.getDeviceID());
    }

}
