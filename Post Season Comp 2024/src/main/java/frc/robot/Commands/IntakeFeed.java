package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;

public class IntakeFeed extends Command {
    public Intake m_intake;

    public IntakeFeed(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setIntakeState(IntakeState.Feeding);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !m_intake.getBeambreak();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakeState(IntakeState.Standby);
    }

}
