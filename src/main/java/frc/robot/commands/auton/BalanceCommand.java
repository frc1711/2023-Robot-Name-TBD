package frc.robot.commands.auton;

import claw.CLAWLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class BalanceCommand extends CommandBase {
    
    private static final CLAWLogger LOG = CLAWLogger.getLogger("commands.balance");
    
    private static final double PITCH_SETPOINT_ERROR_DEG = 1;
    
    private final Swerve swerveDrive;
    
    private final PIDController drivePID = new PIDController(-0.06, 0, -0.008);
    private final Debouncer balancedDebouncer = new Debouncer(1, DebounceType.kRising);
    
    private double initialRobotYaw = 0;
    
    public BalanceCommand (Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        initialRobotYaw = swerveDrive.getRobotYaw();
        LOG.sublog("initialYaw").out(initialRobotYaw+" deg");
        
        drivePID.reset();
        drivePID.setTolerance(1);
    }
    
    @Override
    public void execute () {
        double pitch = swerveDrive.getRobotPitch();
        LOG.sublog("robotPitch").out(pitch+" deg");
        
        double speed = drivePID.calculate(pitch);
        if (Math.abs(pitch) < PITCH_SETPOINT_ERROR_DEG) speed = 0;
        LOG.sublog("speed").out(speed+"");
        
        swerveDrive.moveRobotRelative(new ChassisSpeeds(speed, 0, getTurnCorrection(speed)));
    }
    
    private double getTurnCorrection (double moveSpeed) {
        
        // Don't try to turn if the robot is driving too slowly, it can mess up the balancing process
        double turnSpeed = 0;
        
        LOG.sublog("yaw").out(swerveDrive.getRobotYaw()+" deg");
        if (Math.abs(moveSpeed) > 2) {
            // TODO: Tune turn speed
            turnSpeed = 0;
            // turnSpeed = (initialRobotYaw - swerveDrive.getRobotYaw()) * 0.5;
        }
        
        LOG.sublog("turnSpeed").out(turnSpeed+"");
        return MathUtil.clamp(turnSpeed, -4, 4);
        
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
