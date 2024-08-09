package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    private CANSparkMax climbMotor;
    private ClimbState m_climbState;

    public Climb(int motorId) {
        climbMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        configureClimbMotor();
        m_climbState = ClimbState.Standby;
    }

    public void configureClimbMotor() {
        climbMotor.setIdleMode(IdleMode.kBrake);
        climbMotor.setInverted(true);
    }

    /**
     * sets the current state of the climb, used for moving the climbers up and down
     * 
     * @param climbState
     */
    public void setClimbState(ClimbState climbState) {
        m_climbState = climbState;
        climbMotor.set(m_climbState.climbPercent);
    }

    /**
     * gets the current state of the climb
     * 
     * @return ClimbState
     */
    public ClimbState getClimbState() {
        return m_climbState;
    }

    public enum ClimbState {
        ClimbUp(.5),
        ClimbDown(-.5),
        ClimbHold(0), // TODO: find a way for the climb to calculate the output needed to stay climbed
        Standby(0);

        public double climbPercent;

        private ClimbState(double climbPercent) {
            this.climbPercent = climbPercent;
        }
    }

}
