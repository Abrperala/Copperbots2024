package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LimelightHelpers.LimelightResults;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.lib.util.PathUtils;

public class AllignWithAmp extends Command {
    PathPlannerPath m_path;
    SwerveDrivetrain m_SwerveDrivetrain;
    Limelight m_Limelight;

    public AllignWithAmp(SwerveDrivetrain Swerves, Limelight Limelight) {
        this.m_SwerveDrivetrain = Swerves;
        this.m_Limelight = Limelight;
        addRequirements(m_SwerveDrivetrain, m_Limelight);
    }

    @Override
    public void initialize() {
        m_path = PathUtils.pathFromPoses(m_Limelight.getBluePose2D(), Constants.BLUE_AMP_SCORING_POSITION, true);

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    // gets the correct pose depending on what alliance we are
    public Pose2d getAmpPose2D() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isEmpty())
            return null;
        Alliance allianceColor = ally.get();
        if (allianceColor == Alliance.Red)
            return Constants.RED_AMP_SCORING_POSITION;
        if (allianceColor == Alliance.Blue)
            return Constants.BLUE_AMP_SCORING_POSITION;

        else
            return null;
    }

    @Override
    public String toString() {
        return "I'm an AllignWithAmp!";
    }

}
