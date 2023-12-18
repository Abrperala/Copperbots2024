package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevator extends CommandBase{
    private final Elevator m_elevator;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(30000, 70);
    ProfiledPIDController pid = new ProfiledPIDController(.007, 0.000, 0.01, constraints);

  public SetElevator(Elevator subsystem) {
    m_elevator = subsystem;

    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    pid.reset(m_elevator.getEncoder());
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("PID", pid.calculate(m_elevator.getEncoder(), 31) + .11);
    m_elevator.setElevatorSpeed(pid.calculate(m_elevator.getEncoder(), 31) + .11);

  }

  @Override
  public void end(boolean interrupted){
    m_elevator.setElevatorSpeed(0);
    m_elevator.setBrake();
  }

  
  @Override
  public boolean isFinished() {
    Boolean result = false;
    if (m_elevator.getEncoder() > 31){
      result = true;
    } 
    return result;
  }
}


