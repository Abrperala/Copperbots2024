package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kOrangeTarget = new Color(.4, .4, .1);
    public CANSparkMax intake;

    public Intake() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kOrangeTarget);
        intake = new CANSparkMax(15, MotorType.kBrushed);

    }

    public void setIntakeSpeed(double speed) {
        // intake.set(speed);
    }

    public Color getColorSensor() {
        return m_colorSensor.getColor();
    }

    public String getDetectedColor() {
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(getColorSensor());

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kOrangeTarget) {
            colorString = "Orange";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else {
            colorString = "Unknown";
        }

        return colorString;

    }

    public boolean isNotePresent() {
        return getDetectedColor() == "Orange";
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("detected color", getDetectedColor());
        SmartDashboard.putBoolean("is note present", isNotePresent());
        SmartDashboard.putNumber("blue", getColorSensor().blue);
        SmartDashboard.putNumber("red", getColorSensor().red);
        SmartDashboard.putNumber("green", getColorSensor().green);

    }
}
