package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;

public class elevator extends SubsystemBase {
    //object for detecting the Hall Effect Switch as E brake
    DigitalInput hallEffect = new DigitalInput(0);
    
    // object for limit switch E brake
    DigitalInput limitSwitch = new DigitalInput(5);

    // object for encoder
    Encoder encoder = new Encoder(1, 2, false, CounterBase.EncodingType.k4X);

    public elevator(){
        //encoder.setDistancePerPulse(2/1);
        encoder.reset();

    }
    // method for getting results of the Hall Effect Switch to the dashboard
    // "true" is not tripped
    public boolean getHallEffect(){
        return hallEffect.get();
    }

   // method for getting encoder value
   public Double getEncoder(){
        return (encoder.getDistance()/376);
    }

    // method for getting limit switch
    // "true" is tripped
    public boolean getlimitSwitch(){
        return limitSwitch.get();
    }
    @Override 
    public void periodic(){
       SmartDashboard.putBoolean("Hall Effect: ", getHallEffect());
       SmartDashboard.putNumber("encoder value= ", getEncoder());
       SmartDashboard.putBoolean("limit switch value= ", getlimitSwitch());
    }
}
