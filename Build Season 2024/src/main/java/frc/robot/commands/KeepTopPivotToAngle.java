package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TopPivot;

public class KeepTopPivotToAngle extends Command {

    private final TopPivot m_topPivot;
    private double m_angle;
    private Constraints topConstraints = new Constraints(350, 200);
    private ProfiledPIDController topPID = new ProfiledPIDController(.01, 0, 0, topConstraints);

    public KeepTopPivotToAngle(TopPivot pivot, double angle) {
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
        m_topPivot.setPivot(topPID.calculate(m_topPivot.getPivotAngle(), m_angle));
    }

    @Override
    public boolean isFinished() {
        if (!m_topPivot.isinRange()) {
            return true;

        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_topPivot.setPivot(0);
    }
}
