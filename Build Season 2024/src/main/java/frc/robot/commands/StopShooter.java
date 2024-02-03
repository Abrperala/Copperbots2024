package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {

    private final Shooter m_shooter;

    public StopShooter(Shooter shooter) {
        this.m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooter.setTopShooterSpeed(0);
        m_shooter.setBottomShooterSpeed(0);

    }

    @Override
    public boolean isFinished() {

        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
