package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTables;

public class Wrist extends SubsystemBase {
    private TalonFX wristmotorFx;
    // private CANcoder wristCancoder;
    private WristState wristState;
    private DutyCycleEncoder wristEncoder;
    private ProfiledPIDController wristPID;
    private Constraints wristPIDConstraints;

    // private final MotionMagicVoltage m_mmV = new MotionMagicVoltage(0);
    // private final MotionMagicDutyCycle m_mmDC = new MotionMagicDutyCycle(0);
    // private final PositionDutyCycle m_PDC = new PositionDutyCycle(0);

    public Wrist() {
        wristmotorFx = new TalonFX(18);
        wristEncoder = new DutyCycleEncoder(1);
        wristPIDConstraints = new Constraints(3000, 1200);
        wristPID = new ProfiledPIDController(.012, 0, 0, wristPIDConstraints);
        wristEncoder.setDistancePerRotation(360);
        wristEncoder.setPositionOffset(0.0187);

        // wristCancoder = new CANcoder(23);

        wristmotorFx.getConfigurator().apply(new TalonFXConfiguration());
        // wristCancoder.getConfigurator().apply(new CANcoderConfiguration());

        // /* Configure CANcoder to zero the magnet appropriately */
        // CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        // cc_cfg.MagnetSensor.AbsoluteSensorRange =
        // AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // cc_cfg.MagnetSensor.SensorDirection =
        // SensorDirectionValue.CounterClockwise_Positive;
        // cc_cfg.MagnetSensor.MagnetOffset = -0.179443359375;
        // wristCancoder.getConfigurator().apply(cc_cfg);

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
        // FeedbackConfigs fdb = fx_cfg.Feedback;
        // fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // fdb.FeedbackRemoteSensorID = wristCancoder.getDeviceID();
        // fdb.RotorToSensorRatio = 114.75;
        // fdb.SensorToMechanismRatio = 1;

        // MotionMagicConfigs mm = fx_cfg.MotionMagic;

        // mm.MotionMagicCruiseVelocity = 80; // 5 (mechanism) rotations per second
        // cruise
        // mm.MotionMagicAcceleration = 160; // Take approximately 0.5 seconds to reach
        // max vel
        // // Take approximately 0.1 seconds to reach max accel
        // mm.MotionMagicJerk = 1600;

        // Slot0Configs slot0 = fx_cfg.Slot0;
        // slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0.kP = 6; // A position error of 0.2 rotations results in 12 V output
        // slot0.kI = 0; // No output for integrated error
        // slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

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
     * method to turn wrist motor clockwise at 1 volt
     * 
     * @Check for direction of motor
     */
    public void turnClockwise() {
        wristmotorFx.setVoltage(1);

    }

    /**
     * method to turn wrist motor counterclockwise at 1 volt
     *
     * @Check for direction of motor
     */
    public void turnCounterClockwise() {
        wristmotorFx.setVoltage(-1);

    }

    /**
     * Uses the TalonFX method stopMotor() to stop the motor
     * 
     */
    public void stopTurning() {
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
        Amping(-1);

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
