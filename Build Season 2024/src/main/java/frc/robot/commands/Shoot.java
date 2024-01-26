package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

    private final Shooter m_shooter;
    private double m_speed;

    public Shoot(Shooter shooter, double speed) {
        this.m_shooter = shooter;
        this.m_speed = speed;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooter.shoot(-m_speed);

    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
    }
}
