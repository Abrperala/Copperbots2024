package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Hand subsystem controls a solenoid to extend and retract a piston. it used to be cooler when we had the hand spinning and when the hand closed by motor
 */
public class Hand extends SubsystemBase{
  
  
  
  public boolean pistonState; // Represents the current state of the piston (extended or retracted).
  private DoubleSolenoid m_DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
  
  // retracts the hand on startup
  public Hand(){
    retract();
  }


 /**
   * Extends the piston on the hand, changing {@link #pistonState} to true.
   */
  public void extend() {
    m_DoubleSolenoid.set(Value.kForward);
    pistonState = true;
  } 

  /**
   * Retracts the piston on the hand, changing {@link #pistonState} to false.
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

  @Override
  public void periodic(){

  }


}
