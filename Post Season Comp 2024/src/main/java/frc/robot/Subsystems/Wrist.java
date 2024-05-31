package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTables;

public class Wrist extends SubsystemBase {
    private TalonFX wristmotorFx;

    public Wrist() {
        wristmotorFx = new TalonFX(18);
        wristmotorFx.setNeutralMode(NeutralModeValue.Brake);

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

//   @Override
//     public void periodic() {
//         NetworkTables.updateState("Wrist", getWristState().toString());
//     }

}
