package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

    public Vision() {

    }

    @Override
    public void periodic() {

        double[] a = LimelightHelpers.getT2DArray("limelight-left");
        double thor = a[14];
        double ratio = 14 / thor;
        double totalLength = ratio * 320 / 2;
        double distance = totalLength / Math.tan(Math.toRadians(29.8));
        SmartDashboard.putNumber("distance to note in Inches", distance);
    }

}
