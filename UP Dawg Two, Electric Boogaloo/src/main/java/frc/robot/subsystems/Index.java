package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Index subsystem controls a motor for indexing game pieces, has object detection in terms of beam break
 */
public class Index extends SubsystemBase{
  
  CANSparkMax indexMotor = new CANSparkMax(18, MotorType.kBrushless);
  DigitalInput beamBreak = new DigitalInput(1);
  
   /**
   * Controls the index motor with the specified speed.
   *
   * @param speed The speed at which to control the index motor. Positive values move the mechanism forward (max 1),
   *              while negative values move it backward (max -1).
   */
  public void controlIndex(double speed) {
    indexMotor.set(speed);

  }
  
    /**
   * Returns the state of the beam break sensor.
   *
   * @return False if the beam break sensor is triggered (object detected), true otherwise.
   * 
   * @warning can be tripped by arm
   */
  public boolean getBeamBreak(){
   return beamBreak.get();
  }


  @Override
  public void periodic(){
    SmartDashboard.putBoolean("BeamBreak", beamBreak.get());
  }
}
