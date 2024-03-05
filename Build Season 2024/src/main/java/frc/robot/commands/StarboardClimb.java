package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StarboardClimber;

public class StarboardClimb extends Command {
    private StarboardClimber m_climb;
    private double m_speed;

    public StarboardClimb(StarboardClimber climb, double speed) {
        this.m_climb = climb;
        this.m_speed = speed;
        addRequirements(m_climb);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_climb.setStarboardClimb(m_speed);
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.setStarboardClimb(0);
    }

}
