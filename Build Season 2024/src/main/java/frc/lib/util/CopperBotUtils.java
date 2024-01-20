package frc.lib.util;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public final class CopperBotUtils {

    public static PathPlannerPath pathFromPoses(Pose2d start, Pose2d end, boolean preventFlipping) {

        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not
        // use holonomic rotation.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(start, end);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(2, 1.25, 1 * Math.PI, 0.5 * Math.PI), // The constraints for this path.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation

        );

        path.preventFlipping = preventFlipping;

        return path;
    }

    public static boolean isAllianceBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }

    public static Pose2d getPoseFromATOnAlliance(double apriltag) {

    }

    }

    public static Pose2d getBlueGoalPoseFromAT(double apriltag) {
        switch ((int) apriltag) {
            case 6:
                return Constants.BLUE_AMP_SCORING_POSITION;
            case 9, 10:
                return null; // Phillip is proving a point...
            default:
                return null;
        }
    }

    public static Pose2d getRedGoalPoseFromAT(double apriltag) {
        switch ((int) apriltag) {
            case 5:
                return Constants.RED_AMP_SCORING_POSITION;
            case 9, 10:
                return null; // Phillip is proving a point...
            default:
                return null;
        }
    }

}
