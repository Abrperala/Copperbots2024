package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * command for the robot to rotate to face the cone nodes
 */
public class TurnToAngle extends Command {

  private final SwerveDrivetrain m_drivetrain;
  private DoubleSupplier goalAngle;
  private ChassisSpeeds m_targetSpeeds;
  private PIDController pid;

  public TurnToAngle(SwerveDrivetrain drivetrain, DoubleSupplier targetAngle) {
    this.m_drivetrain = drivetrain;
    this.goalAngle = targetAngle;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  /**
   * sets the rotational speed to the gyro error (Propertional P)
   */
  @Override
  public void execute() {
    pid = new PIDController(.01, 0, 0);
    if (Math.abs(goalAngle.getAsDouble() - m_drivetrain.getPose().getRotation().getDegrees()) < 1) {
      m_targetSpeeds = new ChassisSpeeds();
    } else {
      m_targetSpeeds = new ChassisSpeeds(
          0.0,
          0.0,
          pid.calculate(m_drivetrain.getPose().getRotation().getDegrees(), goalAngle.getAsDouble()));
    }

    m_drivetrain.setChassisSpeeds(m_targetSpeeds);
  }

  @Override
  /**
   * stops the command when the gyro error is within 0.05%
   */
  public boolean isFinished() {
    return false;
    // Math.abs(goalAngle - m_drivetrain.getPose().getRotation().getDegrees()) < 1;
  }

  /**
   * if the command becomes interrupted, the robot goes back to regular drive
   */
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

}
