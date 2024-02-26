package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TopPivot;

public class ManualTopPivot extends Command {

    private final TopPivot m_topPivot;
    private final DoubleSupplier m_JoySupplier;

    public ManualTopPivot(TopPivot pivot, DoubleSupplier joystickSupplier) {
        this.m_topPivot = pivot;
        this.m_JoySupplier = joystickSupplier;
        addRequirements(m_topPivot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_topPivot.setPivot(m_JoySupplier.getAsDouble() * .25);
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
