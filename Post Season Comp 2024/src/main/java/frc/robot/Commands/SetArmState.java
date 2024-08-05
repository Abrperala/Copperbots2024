package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Arm.ArmState;

public class SetArmState extends Command {
    // The subsystem the command operates on
    private final Arm m_arm;
    private final ArmState m_state;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetArmState(Arm arm, ArmState state) {
        m_arm = arm;
        m_state = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setArmState(m_state);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Execution code here
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Return true when the command should end
        return true;
    }
}
