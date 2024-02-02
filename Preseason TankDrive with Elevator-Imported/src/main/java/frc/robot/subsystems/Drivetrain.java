package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    
    final CANSparkMax leftCanSparkMax1 = new CANSparkMax(1, MotorType.kBrushless);
    final CANSparkMax leftCanSparkMax2 = new CANSparkMax(2, MotorType.kBrushless);
    final CANSparkMax rightCanSparkMax1 = new CANSparkMax(3, MotorType.kBrushless);
    final CANSparkMax rightCanSparkMax2 = new CANSparkMax(4, MotorType.kBrushless);


    final DifferentialDrive differentialDrive = new DifferentialDrive(leftCanSparkMax1, rightCanSparkMax1);

    public Drivetrain() {
        differentialDrive.setSafetyEnabled(false); // very good idea 
        leftCanSparkMax2.follow(leftCanSparkMax1);
        rightCanSparkMax2.follow(rightCanSparkMax1);
    }
   
    public void drive(double forward, double rotation) {
        differentialDrive.arcadeDrive(forward, rotation);
    }
}
