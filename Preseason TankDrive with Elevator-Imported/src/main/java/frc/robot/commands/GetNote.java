package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.function.DoubleSupplier;

public class GetNote extends Command {
    
  private final Drivetrain m_drivetrain;
  private final Limelight m_limelight;
  private final ProfiledPIDController pids;
 private final ProfiledPIDController pids2;


  public GetNote(Drivetrain Dsubsystem, Limelight Lsubsystem) {
    m_drivetrain = Dsubsystem;
    m_limelight = Lsubsystem;
    pids = new ProfiledPIDController(.01, .008, 0, new Constraints(1, .5));
    pids2 = new ProfiledPIDController(0, .024,0, new Constraints(1, .5));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_limelight);
  }

@Override
public void initialize(){
    pids.reset(m_limelight.getTX());
    pids2.reset(m_limelight.getTY());
}

  @Override
  public void execute() {
    if (!isInTX()) {
    m_drivetrain.drive(calculateXOutput(), 0);
    }
    else if (!isInTY()){
    m_drivetrain.drive(0, calculateYOutput());
    }
  }


  

@Override
public boolean isFinished(){
    return !m_limelight.getTV();
}

public boolean isInTX(){
    if (Math.abs(m_limelight.getTX()) < 5) {
        return true;
    }
    else {
        return false;
    }
}
 public double calculateXOutput(){
  if (m_limelight.getTX() > 0){
    return -.25;
  }
  else {
    return .25;
  }

 }

public boolean isInTY(){
    if (m_limelight.getTY() < -20) {
        return true;
    }
    else {
        return false;
    }
}
 public double calculateYOutput(){
  if (m_limelight.getTY() > 0){
    return -.3;
  }
  else {
    return -.3;
  }

 }

}

