package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Intake subsystem controls a solenoid to extend and retract an intake.
 */
public class Intake extends SubsystemBase {
  
  public boolean pistonState;
  private DoubleSolenoid m_DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,12, 13);
  
  // retracts intak on startup
  public Intake(){
    retract();
  }

  /**
   * Extends the intake piston, changing the {@link #pistonState} to true.
   */
  public void extend() {
    m_DoubleSolenoid.set(Value.kForward);
    pistonState = true;
  } 

  /**
   * Retracts the intake piston, changing the {@link #pistonState} to false.
   */
  public void retract() {
    m_DoubleSolenoid.set(Value.kReverse);
    pistonState = false;
  }

  /**
   * toggles the arm piston based on {@link #pistonState}
   * prevents extension if the arm is below 45 degrees
   */
  public void togglePiston() {
   if(pistonState) {
    retract();
    } 
  else {
    extend();
   }
  }


}
