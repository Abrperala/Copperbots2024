package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetPoseFromLL extends Command {
    SwerveDrivetrain m_SwerveDrivetrain;
    Limelight m_Limelight;
    boolean isFinished = false;

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
        m_SwerveDrivetrain.resetOdometry(m_Limelight.getBluePose2D());
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
