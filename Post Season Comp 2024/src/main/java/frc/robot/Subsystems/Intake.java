package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeCanSparkMax;
    private IntakeState m_intakeState;

    public Intake() {
        intakeCanSparkMax = new CANSparkMax(15, MotorType.kBrushed);
        m_intakeState = IntakeState.Standby;
    }

    /**
     * sets the state of the intake, used for spinning intake wheels
     * 
     * @param intakeState
     */
    public void setIntakeState(IntakeState intakeState) {
        m_intakeState = intakeState;
        intakeCanSparkMax.set(intakeState.intakePercent);

    }

    /**
     * gets the state of the Intake
     * 
     * @return IntakeState
     */
    public IntakeState getIntakeState() {
        return m_intakeState;
    }

    public enum IntakeState {
        Intaking(.5),
        Outtaking(-.5),
        Feeding(.2),
        Standby(0);

        public double intakePercent;

        private IntakeState(double intakePercent) {
            this.intakePercent = intakePercent;
        };

    }

}
