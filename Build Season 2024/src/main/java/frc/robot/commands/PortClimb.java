package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PortClimber;

public class PortClimb extends Command {
    private PortClimber m_climb;
    private double m_speed;

    public PortClimb(PortClimber climb, double speed) {
        this.m_climb = climb;
        this.m_speed = speed;
        addRequirements(m_climb);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_climb.setPortClimb(m_speed);
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.setPortClimb(0);
    }

}
