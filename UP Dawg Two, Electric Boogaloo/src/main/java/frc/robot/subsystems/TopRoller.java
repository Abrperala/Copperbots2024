package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This subystem is for the top roller of the intake, 
 * I don't remember why this isn't a part of the intake substystem,
 * but changing it is low priority
 */ 
public class TopRoller extends SubsystemBase {
  
  final CANSparkMax topIntake = new CANSparkMax(17, MotorType.kBrushed);

  /**
   * sets the speed of the top intake roller, 
   * @param speed 1 for forward max and -1 for backwards max
   */
  public void rollTopIntake(double speed){
  topIntake.set(speed);
 }

}
