package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command {
    private Climb m_climb;

    public ClimbUp(Climb climb) {
        this.m_climb = climb;
        addRequirements(m_climb);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_climb.setClimb(.7);
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.setClimb(0);
        ;
    }

}
