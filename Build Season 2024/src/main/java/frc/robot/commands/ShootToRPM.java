package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootToRPM extends Command {

    private final Shooter m_shooter;
    private final PIDController m_pid;

    public ShootToRPM(Shooter shooter) {
        m_pid = new PIDController(0, 0, 0);
        this.m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_pid.reset();
    }

    @Override
    public void execute() {
        m_shooter.setTopShooterSpeed(1);
        m_shooter.setBottomShooterSpeed(1);

    }

    @Override
    public boolean isFinished() {

        return m_shooter.shooterAtSpeed();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
