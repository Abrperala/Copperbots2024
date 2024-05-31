package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class SeekNote extends Command {
    public CommandSwerveDrivetrain swerveDrivetrain;
    public SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    public Pose2d botPose;
    public Pose2d notePose;

    public SeekNote(CommandSwerveDrivetrain swerves) {
        this.swerveDrivetrain = swerves;

        addRequirements(swerves);

    }

    @Override
    public void initialize() {
        botPose = new Pose2d();
    }

    @Override
    public void execute() {
        swerveDrivetrain.setControl(request.withRotationalRate(.2 * 1.5 * Math.PI));

        if (LimelightHelpers.getTV("limelight-left")) {
            if (Math.abs(LimelightHelpers.getTX("limelight-left")) < 1) {
                botPose = swerveDrivetrain.getState().Pose;
                // have the botpose, we need someway to find the distance from our camera to the
                // note
                // next we will add or subtract our offset from the limelight from botpose
                // (where it is positioned)

                // pull distance from note to camera,
                // offset camera pose on robot
                // use gyro to find which direction the note is in
                // save note pose somewhere

            }
        }
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = false;
        if (LimelightHelpers.getTV("limelight-left")) {
            if (Math.abs(LimelightHelpers.getTX("limelight-left")) < 1) {
                isFinished = true;
            }
        }

        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrivetrain.setControl(request.withRotationalRate(0));
    }

}
