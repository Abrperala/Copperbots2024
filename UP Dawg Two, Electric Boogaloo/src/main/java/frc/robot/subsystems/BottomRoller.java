package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This subystem is for the bottom roller of the intake, 
 * I don't remember why this isn't a part of the intake substystem,
 * but changing it is low priority
 */ 
public class BottomRoller extends SubsystemBase{
  
  final CANSparkMax bottomIntake = new CANSparkMax(16, MotorType.kBrushed);


  /**
   * sets the speed of the bottom intake roller, 
   * @param speed 1 for forward max and -1 for backwards max
   */
  public void rollBottomIntake(double speed){
    bottomIntake.set(-speed);
   }
   
}
