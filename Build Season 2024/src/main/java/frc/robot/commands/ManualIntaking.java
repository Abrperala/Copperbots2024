package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ManualIntaking extends Command {
    private Intake m_intake;

    public ManualIntaking(Intake intake) {
        m_intake = intake;

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(1);
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakeSpeed(0);
    }
}
