package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class BalanceSlowCommand extends CommandBase {
    
    /**
     * TESTING:
     * 1. Negating yaw offset correction
     * 2. Adjusting drivePID
     */
    
    
    
    private static final double PITCH_SETPOINT_ERROR_DEG = 1;
    
    private final Swerve swerveDrive;
    
    private final PIDController drivePID = new PIDController(-0.049, 0, -0.002);
    private final Debouncer balancedDebouncer = new Debouncer(1, DebounceType.kRising);
    private final SwerveTurnCorrector turnCorrector = new SwerveTurnCorrector();
    
    private Rotation2d initialRobotYaw;
    
    public BalanceSlowCommand (Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        initialRobotYaw = swerveDrive.getRobotRotation();
        
        drivePID.reset();
        
        drivePID.setTolerance(1);
    }
    
    @Override
    public void execute () {
        double pitch = swerveDrive.getRobotPitch();
        
        double speed = drivePID.calculate(pitch);
        if (Math.abs(pitch) < PITCH_SETPOINT_ERROR_DEG) speed = 0;
        
        swerveDrive.moveRobotRelative(
            new ChassisSpeeds(
                speed,
                0,
                turnCorrector.getCorrectionSpeed(swerveDrive.getRobotRotation(), initialRobotYaw)
            )
        );
    }
    
    @Override
    public void end (boolean interrupted) {
        swerveDrive.stop();
    }
    
    @Override
    public boolean isFinished () {
        return balancedDebouncer.calculate(Math.abs(swerveDrive.getRobotPitch()) < PITCH_SETPOINT_ERROR_DEG);
    }
    
}
