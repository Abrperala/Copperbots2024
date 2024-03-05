package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePivot;

public class SetBasePivotToAngle extends Command {

    private final BasePivot m_basePivot;
    private double m_angle;
    private Constraints topConstraints = new Constraints(375, 210);
    private ProfiledPIDController basePID = new ProfiledPIDController(.01, 0, 0, topConstraints);

    public SetBasePivotToAngle(BasePivot pivot, double angle) {
        this.m_basePivot = pivot;
        this.m_angle = angle;
        addRequirements(m_basePivot);
    }

    @Override
    public void initialize() {
        basePID.reset(m_basePivot.getPivotAngle());
    }

    @Override
    public void execute() {
        if (m_basePivot.getPivotAngle() < 90) {
            m_basePivot.setPivot(basePID.calculate(m_basePivot.getPivotAngle(), m_angle));
        } else {
            m_basePivot.setPivot(basePID.calculate(m_basePivot.getPivotAngle(), m_angle));
        }
        //
    }

    @Override
    public boolean isFinished() {

        if (Math.abs(m_angle - m_basePivot.getPivotAngle()) < 2 || !m_basePivot.isinRange()) {
            return true;
        } else {
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_basePivot.setPivot(0);
    }
}
