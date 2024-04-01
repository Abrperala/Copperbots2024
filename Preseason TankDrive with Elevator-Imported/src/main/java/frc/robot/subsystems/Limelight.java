// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
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

    public double getTX() {
        return LimelightHelpers.getTX("");
    }

    public double getTY() {
        return LimelightHelpers.getTY("");
    }

    public double getFid() {
        return LimelightHelpers.getFiducialID("");
    }

    public double getTimeStamp() {
        return Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("") / 1000.0)
                + (LimelightHelpers.getLatency_Capture("") / 1000.0);
    }

    public Pose2d getPose2DFromAlliance() {
        return LimelightHelpers.getBotPose2d_wpiBlue("");
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
        SmartDashboard.putNumber("AprilTag ID #", getFid());
        SmartDashboard.putBoolean("note Present", getTV());
        SmartDashboard.putNumber("error x", getTX());
        SmartDashboard.putNumber("error Y", getTY());
    }

}