package frc.robot.commands;

import java.util.function.DoubleSupplier;

import claw.api.CLAWLogger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.InputCurve.Input2D;
import frc.robot.subsystems.swerve.Swerve;

public class DriveCommand extends CommandBase {
    
    private static final CLAWLogger LOG = CLAWLogger.getLogger("commands.drive");
    
    private static final InputCurve
        ROTATE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14),
        STRAFE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14);
    
    private final Swerve swerve;
    private final DoubleSupplier strafeXAxis, strafeYAxis, rotateAxis;
    
    public DriveCommand (Swerve swerve, DoubleSupplier strafeXAxis, DoubleSupplier strafeYAxis, DoubleSupplier rotateAxis) {
        this.swerve = swerve;
        this.strafeXAxis = strafeXAxis;
        this.strafeYAxis = strafeYAxis;
        this.rotateAxis = rotateAxis;
        addRequirements(swerve);
        
        // Add functionality which isn't button-bound to the shuffleboard
        RobotContainer.putSendable("Zero Swerve Modules", new InstantCommand(() -> swerve.zeroModules(), swerve));
        RobotContainer.putSendable("Zero Gyro", new InstantCommand(() -> swerve.zeroGyro(), swerve));
    }
    
    @Override
    public void initialize () {
        swerve.stop();
    }
    
    @Override
    public void execute () {
        
        // Apply the input curves
        Input2D strafeInputRaw = new Input2D(strafeXAxis.getAsDouble(), strafeYAxis.getAsDouble());
        double rotateInputRaw = rotateAxis.getAsDouble();
        
        Input2D strafeSpeeds = InputCurve.apply(STRAFE_CURVE, strafeInputRaw).scale(5);
        double rotateSpeed = InputCurve.apply(ROTATE_CURVE, rotateInputRaw) * 6;
        
        LOG.sublog("input").sublog("strafe").sublog("x").out(strafeInputRaw.x()+"");
        LOG.sublog("input").sublog("strafe").sublog("y").out(strafeInputRaw.y()+"");
        LOG.sublog("input").sublog("rotate").out(rotateInputRaw+"");
        
        LOG.sublog("inputAdj").sublog("strafe").sublog("x").out(strafeSpeeds.x()+"");
        LOG.sublog("inputAdj").sublog("strafe").sublog("y").out(strafeSpeeds.y()+"");
        LOG.sublog("inputAdj").sublog("rotate").out(rotateSpeed+"");
        
        // Robot orientation and ChassisSpeeds is based on the idea that +x is the front of the robot,
        // +y is the left side of the robot, etc.
        // Axes, of course, do not work like this
        swerve.moveRobotRelative(new ChassisSpeeds(-strafeSpeeds.y(), -strafeSpeeds.x(), rotateSpeed));
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
}
