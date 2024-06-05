// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

public class DriveToPose extends Command {
    /** Creates a new DriveToPose. */
    Command pathfindingCommand;
    CommandSwerveDrivetrain swerve;

    public DriveToPose(CommandSwerveDrivetrain swerve, Pose2d desiredPose) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.swerve = swerve;
        pathfindingCommand = swerve.driveToPose(desiredPose);
        addRequirements(swerve);
    }

    public DriveToPose(CommandSwerveDrivetrain swerve) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.swerve = swerve;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        if (swerve.getNotePose().getX() != 0 && swerve.getNotePose().getY() != 0) {
            pathfindingCommand = swerve.driveToPose(swerve.getNotePose());
        } else {
            pathfindingCommand = swerve.driveToPose(swerve.getState().Pose);
        }

        pathfindingCommand.schedule();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pathfindingCommand.isFinished();
    }
}