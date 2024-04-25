package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootToRPM extends Command {

    private final Shooter m_shooter;

    public ShootToRPM(Shooter shooter) {
        this.m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooter.shooterRun(85);

    }

    @Override
    public boolean isFinished() {

        return m_shooter.shooterAtSpeed();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
