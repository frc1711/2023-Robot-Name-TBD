package frc.robot.commands.auton;

import claw.api.CLAWLogger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.InputCurve;
import frc.robot.subsystems.swerve.Swerve;

public class Balance extends CommandBase {
    
    private static final CLAWLogger LOG = CLAWLogger.getLogger("commands.balance");
    private final Swerve swerve;
    private double startGyroYaw;
    
    public Balance (Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
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
        
        double driveSpeed = getDriveSpeed(swerve.getRobotPitch());
        LOG.sublog("driveSpeed").out(driveSpeed+"");
        
        LOG.sublog("yaw").out(swerve.getRobotYaw()+"");
        
        double angleOffset = startGyroYaw - swerve.getRobotYaw();
        LOG.sublog("yawOffset").out(angleOffset+"");
        
        double turnCorrection = InputCurve.apply(InputCurve.NO_CURVE.withDeadband(2./15.), angleOffset/15) * 14;
        LOG.sublog("turnCorrection").out(turnCorrection+"");
        
        swerve.moveRobotRelative(new ChassisSpeeds(driveSpeed, 0, turnCorrection));
    }
    
    private static final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.25, 1);
    
    private static final InputCurve speedCurve = ((InputCurve)((double input) -> Math.pow(input, 3.5))).withDeadband(1./14.);
    
    private static double getDriveSpeed (double pitch) {
        double inputAngleVal = pitch / 14;
        double appliedValue = InputCurve.apply(speedCurve, inputAngleVal) * 1.;
        return driveFeedforward.calculate(appliedValue);
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
}
