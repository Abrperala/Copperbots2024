// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.util.CopperBotUtils;
import frc.lib.util.LimelightHelpers;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    /** Creates a new Limelight. */
    public Limelight() {
    }

    public boolean getTV() {
        return LimelightHelpers.getTV("");
    }

    public double getFid() {
        return LimelightHelpers.getFiducialID("");
    }

    public double getTimeStamp() {
        return Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("") / 1000.0)
                + (LimelightHelpers.getLatency_Capture("") / 1000.0);
    }

    public Pose2d getPose2DFromAlliance() {
        if (CopperBotUtils.isAllianceBlue()) {
            return LimelightHelpers.getBotPose2d_wpiBlue("");

        } else {
            return LimelightHelpers.getBotPose2d_wpiRed("");
        }
    }

    public double getX() {
        return getPose2DFromAlliance().getX();
    }

    public double getY() {
        return getPose2DFromAlliance().getY();
    }

    public double getRotation() {
        return getPose2DFromAlliance().getRotation().getDegrees();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pose x", getX());
        SmartDashboard.putNumber("pose y", getY());
        SmartDashboard.putNumber("LL rot", getRotation());
        SmartDashboard.putNumber("AprilTag ID #", getFid());
    }

}