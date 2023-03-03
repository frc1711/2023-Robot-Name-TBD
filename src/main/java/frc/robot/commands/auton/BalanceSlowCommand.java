package frc.robot.commands.auton;

import claw.math.Transform;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    
    private final PIDController drivePID = new PIDController(-0.05, 0, -0.002);
    
    private final Debouncer balancedDebouncer = new Debouncer(1, DebounceType.kRising);
    
    private double initialRobotYaw = 0;
    
    private final Transform yawOffsetToCorrectionTurn =
        // Wrap degrees from -180 to +180
        ((Transform)(deg -> {
            while (deg > 180) deg -= 360;
            while (deg < -180) deg += 360;
            return deg;
        }))
        
        // .then(Transform.NEGATE)
        
        // Apply the corrective turn
        .then(offsetDeg -> offsetDeg / 30.)
        .then(Transform.clamp(-1, 1))
        .then(v -> 3.5*v);
    
    public BalanceSlowCommand (Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        initialRobotYaw = swerveDrive.getRobotYaw();
        
        drivePID.reset();
        
        drivePID.setTolerance(1);
    }
    
    @Override
    public void execute () {
        double pitch = swerveDrive.getRobotPitch();
        
        double speed = drivePID.calculate(pitch);
        if (Math.abs(pitch) < PITCH_SETPOINT_ERROR_DEG) speed = 0;
        
        double turnSpeed = yawOffsetToCorrectionTurn.apply(swerveDrive.getRobotYaw() - initialRobotYaw);
        
        swerveDrive.moveRobotRelative(new ChassisSpeeds(speed, 0, turnSpeed));
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
