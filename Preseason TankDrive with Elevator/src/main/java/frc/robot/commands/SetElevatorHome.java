package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Elevator;

public class SetElevatorHome extends CommandBase {
  private final Elevator m_elevator;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(30000, 70);
  ProfiledPIDController pid = new ProfiledPIDController(.2, 0.000, 0.0003, constraints);

  public SetElevatorHome(Elevator subsystem) {
  m_elevator = subsystem;


  addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
  pid.reset(m_elevator.getEncoder());
  }

  @Override
  public void execute() {
  m_elevator.setElevatorSpeed(pid.calculate(m_elevator.getEncoder(), 0));
  }

  @Override
  public void end(boolean interrupted){
  m_elevator.setElevatorSpeed(0);
  }


  @Override
  public boolean isFinished() {
  Boolean result = false;
  if (m_elevator.getEncoder() < 1){
    result = true;
  } 
  return result;
  }
}




