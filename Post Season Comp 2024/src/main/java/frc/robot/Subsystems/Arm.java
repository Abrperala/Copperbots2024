package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private TalonFX leftarmmotorFX;
    private TalonFX rightarmmotorFX;

    public Arm() {
        leftarmmotorFX = new TalonFX(16);
        rightarmmotorFX = new TalonFX(17);

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

}