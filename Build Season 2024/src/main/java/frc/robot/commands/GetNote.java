package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class GetNote extends Command {

    private final SwerveDrivetrain m_drivetrain;
    private final ProfiledPIDController pid = new ProfiledPIDController(.1, 0, 0, new Constraints(4, 3));
    private final ProfiledPIDController pid2 = new ProfiledPIDController(.04, 0, 0, new Constraints(4, 3));

    private ChassisSpeeds m_targetSpeeds;

    public GetNote(SwerveDrivetrain Dsubsystem) {
        m_drivetrain = Dsubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        pid.reset(m_drivetrain.m_limeLight.getNoteTY());
        pid2.reset(m_drivetrain.m_limeLight.getNoteTX());

    }

    @Override
    public void execute() {
        if (!(Math.abs(m_drivetrain.m_limeLight.getNoteTX()) < 5)) {
            m_targetSpeeds = new ChassisSpeeds(
                    1,
                    0,
                    calculateXOutput());
        } else {
            m_targetSpeeds = new ChassisSpeeds(
                    1,
                    0,
                    0);
        }

        m_drivetrain.setChassisSpeeds(m_targetSpeeds);
    }

    @Override
    public boolean isFinished() {
        return !m_drivetrain.m_limeLight.getNoteTV();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    public double calculateXOutput() {
        if (m_drivetrain.m_limeLight.getNoteTX() > 0) {
            return -.5;
        } else {
            return .5;
        }
    }

}
