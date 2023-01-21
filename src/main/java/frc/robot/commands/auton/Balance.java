package frc.robot.commands.auton;

import claw.api.CLAWLogger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.InputCurve;
import frc.robot.subsystems.swerve.Swerve;

public class Balance extends CommandBase {
    
    private static final CLAWLogger LOG = CLAWLogger.getLogger("commands.balance");
    
    private final Swerve swerve;
    private double startGyroYaw;
    
    private final PIDController drivePID = new PIDController(0.04, 0, 0.008);
    
    public Balance (Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        
        drivePID.setTolerance(1);
    }
    
    @Override
    public void initialize () {
        swerve.stop();
        startGyroYaw = swerve.getRobotYaw();
        LOG.sublog("yawInitial").out(startGyroYaw+"");
    }
    
    @Override
    public void execute () {
        LOG.sublog("pitch").out(swerve.getRobotPitch()+"");
        
        double driveSpeed = -drivePID.calculate(swerve.getRobotPitch(), 0);
        LOG.sublog("driveSpeed").out(driveSpeed+"");
        
        LOG.sublog("yaw").out(swerve.getRobotYaw()+"");
        
        double angleOffset = startGyroYaw - swerve.getRobotYaw();
        LOG.sublog("yawOffset").out(angleOffset+"");
        
        double turnCorrection = InputCurve.apply(InputCurve.NO_CURVE.withDeadband(2./15.), angleOffset/15) * 14;
        LOG.sublog("turnCorrection").out(turnCorrection+"");
        
        swerve.moveRobotRelative(new ChassisSpeeds(driveSpeed, 0, turnCorrection));
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
}
