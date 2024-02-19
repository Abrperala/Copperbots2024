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
    private final SimpleMotorFeedforward feedforward;

    public ShootToRPM(Shooter shooter) {
        feedforward = new SimpleMotorFeedforward(.005, 1 / Constants.SHOOTER_FREE_RPM);
        this.m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooter.setTopShooterSpeed(feedforward.calculate(Constants.SHOOTER_TARGET_RPM));
        m_shooter.setBottomShooterSpeed(feedforward.calculate(Constants.SHOOTER_TARGET_RPM));

    }

    @Override
    public boolean isFinished() {

        return m_shooter.shooterAtSpeed();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
