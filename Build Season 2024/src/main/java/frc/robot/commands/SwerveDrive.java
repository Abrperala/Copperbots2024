package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveDrive extends CommandBase {

    private SwerveDrivetrain m_drivetrain;    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;
    private BooleanSupplier m_robotCentricSup;
    private SlewRateLimiter m_xAxisARateLimiter;
    private SlewRateLimiter m_yAxisARateLimiter;

    public SwerveDrive(SwerveDrivetrain drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {

        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);

        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
        m_robotCentricSup = robotCentricSup;

        m_xAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        m_yAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);

    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double xAxis = MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double yAxis = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rAxis = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Square joystick inputs */
        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        /* Filter joystick inputs using slew rate limiter */
        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        /* Drive */ 
        m_drivetrain.drive(
            new Translation2d(xAxisFiltered, yAxisFiltered).times(Constants.MAX_SPEED), 
            rAxisSquared * Constants.MAX_ANGULAR_VELOCITY, 
            !m_robotCentricSup.getAsBoolean(), 
            true
        );

    }
}