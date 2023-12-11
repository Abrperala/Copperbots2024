package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class StickDrive extends CommandBase {
    
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;

  public StickDrive(Drivetrain subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    m_drivetrain = subsystem;
    m_forward = forward;
    m_rotation = rotation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  @Override
  public void execute() {
     m_drivetrain.drive(m_forward.getAsDouble(), m_rotation.getAsDouble());
  }



}
