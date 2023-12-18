package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.CounterBase;

public class Elevator extends SubsystemBase {
    //object for detecting the Hall Effect Switch as E brake
    DigitalInput hallEffect = new DigitalInput(0);
    
    // object for limit switch E brake
    DigitalInput limitSwitch = new DigitalInput(5);

    // object for encoder
    Encoder encoder = new Encoder(1, 2, false, CounterBase.EncodingType.k4X);

    CANSparkMax elevator1 = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax elevator2 = new CANSparkMax(6, MotorType.kBrushless);

    public Elevator(){
        encoder.reset();

    }
    // method for getting results of the Hall Effect Switch to the dashboard
    // "true" is tripped
    public boolean getHallEffect(){
        return !hallEffect.get();
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

    public void setBrake(){
        elevator1.setIdleMode(IdleMode.kBrake);
        elevator2.setIdleMode(IdleMode.kCoast);
    }


    /** Sets the speed of the elevators if eStops are not tripped
     * @param speed
     */ 
    public void setElevatorSpeed(double speed){
        boolean estop;
        if(getlimitSwitch() || getHallEffect() ){
            elevator1.set(0);
            elevator2.set(0);
            estop = true;
        }
        else{
            elevator1.set(speed);
            elevator2.set(speed);
            estop = false;
                //TODO: make logic so the elevator can't get perma stunned if costantly tripped
                // or make a manual control (probably bad idea)
    
            
        }
        SmartDashboard.putBoolean("Estop Tripped?", estop);

    }
    @Override 
    public void periodic(){
       SmartDashboard.putBoolean("Hall Effect:", getHallEffect());
       SmartDashboard.putNumber("encoder value:", getEncoder());
       SmartDashboard.putBoolean("limit switch value:", getlimitSwitch());
    }
}
