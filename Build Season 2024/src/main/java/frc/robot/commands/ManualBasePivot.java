package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePivot;
import frc.robot.subsystems.TopPivot;

public class ManualBasePivot extends Command {

    private final BasePivot m_basePivot;
    private final DoubleSupplier m_JoySupplier;

    public ManualBasePivot(BasePivot pivot, DoubleSupplier joystickSupplier) {
        this.m_basePivot = pivot;
        this.m_JoySupplier = joystickSupplier;
        addRequirements(m_basePivot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_basePivot.setPivot(m_JoySupplier.getAsDouble() * .25);
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
