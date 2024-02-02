package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Intake extends Command {

    private final Shooter m_shooter;

    public Intake(Shooter shooter) {
        this.m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooter.intake();
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
