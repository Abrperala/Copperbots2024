package frc.robot.commands;

import frc.lib.util.GeneralUtils;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrive extends Command {

    private SwerveDrivetrain m_swerveDrivetrain;
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;
    private BooleanSupplier m_robotCentricSup;
    private SlewRateLimiter m_xAxisLimiter;
    private SlewRateLimiter m_yAxisLimiter;
    private BooleanSupplier m_isEvading;
    private BooleanSupplier m_isLocked;
    private BooleanSupplier m_isRotatingFast;
    private boolean limitSpeed = false;

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier isEvading,
            BooleanSupplier isLocked,
            BooleanSupplier isRotatingFast) {

        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
        m_robotCentricSup = robotCentricSup;
        m_isEvading = isEvading;
        m_isLocked = isLocked;
        m_isRotatingFast = isRotatingFast;

        m_xAxisLimiter = new SlewRateLimiter(Constants.RATE_LIMITER);
        m_yAxisLimiter = new SlewRateLimiter(Constants.RATE_LIMITER);

    }

    @Override
    public void execute() {

        double rotationScalar = 1;

        if (m_isRotatingFast.getAsBoolean() && !limitSpeed) {

            limitSpeed = true;
        } else {

            limitSpeed = false;
        }

        double xAxis;
        double yAxis;
        double rAxis;
        if (limitSpeed) {
            xAxis = MathUtil.applyDeadband(m_translationSup.getAsDouble() * .5, Constants.STICK_DEADBAND);
            yAxis = MathUtil.applyDeadband(m_strafeSup.getAsDouble() * .5, Constants.STICK_DEADBAND);
            rAxis = MathUtil.applyDeadband(m_rotationSup.getAsDouble() * .5, Constants.STICK_DEADBAND);

        } else {
            xAxis = MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            yAxis = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
            rAxis = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        }

        double xAxisSquared = xAxis * xAxis * Math.signum(xAxis);
        double yAxisSquared = yAxis * yAxis * Math.signum(yAxis);
        double rAxisSquared = rAxis * rAxis * Math.signum(rAxis) * rotationScalar;

        double xAxisFiltered = m_xAxisLimiter.calculate(xAxisSquared);
        double yAxisFiltered = m_yAxisLimiter.calculate(yAxisSquared);
        if (!GeneralUtils.isAllianceBlue()) {
            xAxisFiltered = -xAxisFiltered;
            yAxisFiltered = -yAxisFiltered;
        }

        /* Drive */
        m_swerveDrivetrain.drive(
                new Translation2d(-xAxisFiltered, -yAxisFiltered).times(Constants.MAX_SPEED),
                -rAxisSquared * Constants.MAX_ANGULAR_VELOCITY,
                !m_robotCentricSup.getAsBoolean(),
                true,
                m_isEvading.getAsBoolean(),
                m_isLocked.getAsBoolean());
    }
}