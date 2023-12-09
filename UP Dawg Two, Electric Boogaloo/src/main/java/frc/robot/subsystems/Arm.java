package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The Arm subsystem controls the movement of the arm using a motor, encoder, and limit switch.
 * It also manages a pneumatic piston for extending and retracting the arm.
 */
public class Arm extends SubsystemBase {

  private final CANSparkMax arm1 = new CANSparkMax(13, MotorType.kBrushless);
  private final Encoder armEncoder = new Encoder(8, 7, true, CounterBase.EncodingType.k4X);
  private final DigitalInput limitSwitch = new DigitalInput(2);
  private  final DoubleSolenoid m_DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 11);
  public boolean pistonState; // variable to know if piston is extended, if true piston is extended

  // configs encoder and retracts the arm on startup
  public Arm(){
    armEncoder.setDistancePerPulse(360/2048.0); // converts encoder units to degrees moved on arm, from output shaft
    armEncoder.reset();
    retract();
  }

  /**
   * Sets the speed of the arm motor.
   *
   * @param speed 1 for forward max and -1 for backward max.
   */
  public void setArmPosition(double speed){
    arm1.set(speed);
  }


  /**
   * resets the arm encoder
   * used to reset to zero if arm hits limit switch
   */
  public void resetEncoder(){
    armEncoder.reset();
  }

  /**
   * Returns the value of the arm encoder.
   *
   * @return Degrees of the arm, where straight down is 0,
   * and the arm outwards is a positive increase.
   */
  public double getEncoderDistance(){
    return armEncoder.getDistance();
  }
   
  /**
   * Extends the piston on the arm, changing {@link #pistonState} to true.
   */
  public void extend() {
    m_DoubleSolenoid.set(Value.kForward);
    pistonState = true;
  }


  /**
   * Retracts the piston on the arm, changing {@link #pistonState} to false.
   */
  public void retract() {
    m_DoubleSolenoid.set(Value.kReverse);
    pistonState = false;
  }

  /**
   * Returns true if the limit switch is tripped, false if untripped.
   *
   * @return The state of the limit switch.
   */
  public boolean getLimitSwitch(){
    return limitSwitch.get();
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
      if(armEncoder.getDistance() >= 45){
        extend();
    }
  }
}

  @Override
  public void periodic(){
    SmartDashboard.putNumber("arm encoder", armEncoder.getDistance());
    SmartDashboard.putBoolean("limit switch", limitSwitch.get());
  }

}

