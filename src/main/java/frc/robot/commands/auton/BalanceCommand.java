package frc.robot.commands.auton;

import claw.math.Transform;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class BalanceCommand extends CommandBase {
    
    
    // TODO: Add balance command to the drive remote
    
    
    
    private static final double PITCH_SETPOINT_ERROR_DEG = 1;
    
    private final Swerve swerveDrive;
    
    private final PIDController
        drivePID = new PIDController(-0.086, 0, -0.002),
        correctiveTurnPID = new PIDController(0.02, 0, 0);
    
    private final Debouncer balancedDebouncer = new Debouncer(1, DebounceType.kRising);
    
    private double initialRobotYaw = 0;
    
    private final Transform yawOffsetToCorrectionTurn =
        // Wrap degrees from -180 to +180
        ((Transform)(deg -> {
            while (deg > 180) deg -= 360;
            while (deg < -180) deg += 360;
            return deg;
        }))
        
        // Apply the corrective turn PID
        .then(correctiveTurnPID::calculate)
        
        // Apply a clamp
        .then(Transform.clamp(-2, 2));
    
    public BalanceCommand (Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        initialRobotYaw = swerveDrive.getRobotYaw();
        
        drivePID.reset();
        correctiveTurnPID.reset();
        
        drivePID.setTolerance(1);
        correctiveTurnPID.setTolerance(3);
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
