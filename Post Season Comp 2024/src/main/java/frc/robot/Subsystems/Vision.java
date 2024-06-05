package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

    public Vision() {

    }

    public static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    /**
     * gets the distance from the note to the camera in inches
     * 
     * @return distance from the note to the camera
     */
    public double getDistanceFromNote() {
        if (LimelightHelpers.getTV("limelight-left")) {

            double[] a = LimelightHelpers.getT2DArray("limelight-left");
            double thor = a[14];
            double ratio = Constants.noteLength / thor;
            double totalLength = ratio * 1280 / 2;
            double distance = totalLength / Math.tan(Math.toRadians(29.8));
            return distance;
        } else {
            return 0;
        }

    }

    /**
     * gets the distance from the note to the center of the robot in inches
     * 
     * @return distance from the note to the center of the robot in inches
     */
    public double getDistanceFromNoteToRobot() {
        double result = 0;
        if (LimelightHelpers.getTV("limelight-left")) {
            result = Math.sqrt((getDistanceFromNote() * getDistanceFromNote()) - 19 * 19);
            result = result + 14;
        }
        return result;
    }

    public double getTx() {
        return LimelightHelpers.getTX("limelight-left");
    }

    public boolean getTv() {
        return LimelightHelpers.getTV("limelight-left");
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("distance from note to camera", getDistanceFromNote());
        SmartDashboard.putNumber("distance from robot to note", getDistanceFromNoteToRobot());
        SmartDashboard.putBoolean("noteseen", getTv());
    }

}
