package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetPoseFromLL extends Command {
    private SwerveDrivetrain m_SwerveDrivetrain;
    private Limelight m_Limelight;
    private boolean isFinished = false;

    public ResetPoseFromLL(Limelight limelight, SwerveDrivetrain swerves) {
        this.m_Limelight = limelight;
        this.m_SwerveDrivetrain = swerves;
        addRequirements(m_Limelight, m_SwerveDrivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_Limelight.hasTargetAprilTag()) {
            m_SwerveDrivetrain.resetOdometry(m_Limelight.getPose2DFromAlliance());
        }
        isFinished = true;
    }

    @Override
    public boolean isFinished() {

        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
