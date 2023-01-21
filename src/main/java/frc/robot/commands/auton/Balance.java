package frc.robot.commands.auton;

import claw.api.CLAWLogger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.InputCurve;
import frc.robot.subsystems.swerve.Swerve;

public class Balance extends CommandBase {
    
    private static final CLAWLogger LOG = CLAWLogger.getLogger("commands.balance");
    
    private final Swerve swerve;
    private double startGyroYaw;
    
    private static final InputCurve BASE_SPEED_CURVE = (double x) -> Math.pow(x, 3);
    private static final InputCurve MOMENTUM_CANCEL_CURVE = (double x) -> Math.pow(x, 0.8);
    
    private final AutobalanceFeedforward driveAutobalance = new AutobalanceFeedforward(
        angle -> InputCurve.apply(BASE_SPEED_CURVE, angle / 8.) * 1.1,
        speed -> InputCurve.apply(MOMENTUM_CANCEL_CURVE, speed / 60) * 2
    );
    
    public Balance (Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        swerve.stop();
        startGyroYaw = swerve.getRobotYaw();
        LOG.sublog("yawInitial").out(startGyroYaw+"");
        driveAutobalance.reset();
        driveAutobalance.setTolerance(5);
    }
    
    @Override
    public void execute () {
        LOG.sublog("pitch").out(swerve.getRobotPitch()+"");
        
        double driveSpeed = driveAutobalance.calculate(swerve.getRobotPitch());
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
