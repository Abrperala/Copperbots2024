package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTables;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeCanSparkMax;// declare an object that creates an intake motor controller
    private IntakeState m_intakeState;// declare an object/variable that enumarates the 4 possible intake states.
    private DigitalInput beambreak; // declare an object that creates an input on DIO port 2.

    public Intake() {
        intakeCanSparkMax = new CANSparkMax(15, MotorType.kBrushed);
        m_intakeState = IntakeState.Standby;
        beambreak = new DigitalInput(2);
    }

    // Create a method (getBeambreak) to check the state of beambreak (DIO port 2).
    /**
     * returns true when the beam break is tripped, false when not
     * 
     * @return boolean of beam break tripped
     */
    public boolean getBeambreak() {

        return !beambreak.get();
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
     * returns the current state of the intake
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

    @Override
    public void periodic() {
        NetworkTables.updateState("Intake", getIntakeState().toString());
        SmartDashboard.putBoolean("Intake Beambreak", getBeambreak());// puts the status of the beam break on the
                                                                      // dashboard
    }

}
