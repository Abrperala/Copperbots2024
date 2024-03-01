// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.util.GeneralUtils;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    /** Creates a new Limelight. */
    public Limelight() {
    }

    public static Limelight instance;

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
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
        if (GeneralUtils.isAllianceBlue()) {
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

    public boolean hasTargetAprilTag() {
        if (GeneralUtils.isAllianceBlue()) {
            if (getFid() == 1 || getFid() == 2 || getFid() == 6 || getFid() == 7 || getFid() == 8) {
                return true;
            } else {
                return false;
            }
        } else {
            if (getFid() == 5 || getFid() == 9 || getFid() == 10) {
                return true;
            } else {
                return false;
            }
        }
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("pose x", getX());
        // SmartDashboard.putNumber("pose y", getY());
        // SmartDashboard.putNumber("LL rot", getRotation());
        SmartDashboard.putNumber("AprilTag ID #", getFid());
        SmartDashboard.putBoolean("Apriltag Present", hasTargetAprilTag());
    }

}