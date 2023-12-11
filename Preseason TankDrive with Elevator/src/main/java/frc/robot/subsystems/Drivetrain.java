package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    
    final CANSparkMax leftCanSparkMax1 = new CANSparkMax(1, MotorType.kBrushless);
    final CANSparkMax leftCanSparkMax2 = new CANSparkMax(2, MotorType.kBrushless);
    final CANSparkMax rightCanSparkMax1 = new CANSparkMax(3, MotorType.kBrushless);
    final CANSparkMax rightCanSparkMax2 = new CANSparkMax(4, MotorType.kBrushless);

    final MotorControllerGroup leftMotors = new MotorControllerGroup(leftCanSparkMax1, leftCanSparkMax2);
    final MotorControllerGroup rightMotors = new MotorControllerGroup(rightCanSparkMax1, rightCanSparkMax2);

    final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    public Drivetrain() {
        differentialDrive.setSafetyEnabled(false); // very good idea 
    }
   
    public void drive(double forward, double rotation) {
        differentialDrive.arcadeDrive(forward, rotation);
    }
}
