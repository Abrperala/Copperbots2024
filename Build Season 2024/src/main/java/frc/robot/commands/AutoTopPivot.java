package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TopPivot;

public class AutoTopPivot extends Command {

    private final TopPivot m_topPivot;
    private DoubleSupplier m_angle;
    private Constraints topConstraints = new Constraints(1000, 350);
    private ProfiledPIDController topPID = new ProfiledPIDController(.012, 0.0, 0, topConstraints);

    public AutoTopPivot(TopPivot pivot, DoubleSupplier angle) {
        this.m_topPivot = pivot;
        this.m_angle = angle;
        addRequirements(m_topPivot);
    }

    @Override
    public void initialize() {
        topPID.reset(m_topPivot.getPivotAngle());
    }

    @Override
    public void execute() {
        m_topPivot.setPivot(topPID.calculate(m_topPivot.getPivotAngle(), m_angle.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_topPivot.setPivot(0);
    }
}
