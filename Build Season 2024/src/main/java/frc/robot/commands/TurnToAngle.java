package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.GeneralUtils;
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
    double currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
    if (!GeneralUtils.isAllianceBlue()) {
      currentAngle = m_drivetrain.m_limeLight.getRedDegrees();
    }
    pid = new PIDController(.010, 0.01, 0);
    if (Math.abs(goalAngle.getAsDouble() - currentAngle) < 1) {
      m_targetSpeeds = new ChassisSpeeds();
    } else {
      m_targetSpeeds = new ChassisSpeeds(
          0.0,
          0.0,
          pid.calculate(currentAngle, goalAngle.getAsDouble()));
    }

    m_drivetrain.setChassisSpeeds(m_targetSpeeds);
  }

  @Override

  public boolean isFinished() {
    // return Math.abs(goalAngle.getAsDouble() -
    // m_drivetrain.getPose().getRotation().getDegrees()) < 6;
    return false;
  }

  /**
   * if the command becomes interrupted, the robot goes back to regular drive
   */
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

}