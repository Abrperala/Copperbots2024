package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.commands.SwerveDrive;

public class AlignWithTarget extends CommandBase {
    private final LimelightSubsystem m_limelight;
    private final SwerveDrivetrain m_swerve;

    public AlignWithTarget(LimelightSubsystem limelight, SwerveDrivetrain swerve) {
      this.m_limelight = limelight;
      this.m_swerve = swerve;
      addRequirements(m_limelight, m_swerve);
    }
  
  
    @Override
    public void initialize() {
      
    }
  
    @Override
    public void execute() {
        if (m_limelight.hasTarget()){
            if (!m_limelight.inPositionX()){
                m_swerve.drive(
                new Translation2d(m_limelight.getPercentErrorX(), 0),
                0,
                true,
                true
                );
        }}

        else {
         System.out.println("No Target"); 
        }
     }
    
  
  
    @Override
    public boolean isFinished() {
        return m_limelight.inPositionX();
    }
  
  
  
    @Override
    public void end(boolean interrupted) {
    
    }
  }


