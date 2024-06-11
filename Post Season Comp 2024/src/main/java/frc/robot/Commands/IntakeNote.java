// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;

public class IntakeNote extends Command {
    /** Creates a new IntakeNote. */
    Command pathfindingCommand;
    CommandSwerveDrivetrain swerve;
    Intake m_intake;

    public IntakeNote(CommandSwerveDrivetrain swerve, Intake intake, Pose2d desiredPose) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.swerve = swerve;
        m_intake = intake;
        pathfindingCommand = new ParallelCommandGroup(swerve.driveToPose(desiredPose),
                new IntakeUntilTripped(m_intake));
        addRequirements(swerve, m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (swerve.getNotePose().getX() != 0 && swerve.getNotePose().getY() != 0) {
            pathfindingCommand = new ParallelCommandGroup(swerve.driveToPose(swerve.getNotePose()),
                    new IntakeUntilTripped(m_intake));
        } else {
            pathfindingCommand = new PrintCommand("no Note Found");
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