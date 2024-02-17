package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePivot;

public class SetBasePivotToAngle extends Command {

    private final BasePivot m_basePivot;
    private double m_angle;
    private Constraints topConstraints = new Constraints(50, 35);
    private ProfiledPIDController basePID = new ProfiledPIDController(.1, 0, 0, topConstraints);

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
        m_basePivot.setPivot(.105 + basePID.calculate(m_basePivot.getPivotAngle(), m_angle));
        //
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_basePivot.setPivot(0);
    }
}
