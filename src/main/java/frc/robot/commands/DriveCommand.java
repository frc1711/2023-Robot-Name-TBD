package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.InputCurve.Input2D;
import frc.robot.subsystems.swerve.Swerve;

public class DriveCommand extends CommandBase {
    
    private static final InputCurve
        ROTATE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14),
        STRAFE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14);
    
    private final Swerve swerve;
    private final BooleanSupplier xModeInput;
    private final DoubleSupplier strafeXAxis, strafeYAxis, rotateAxis;
    
    private double DS_strafeX, DS_strafeY, DS_rotate;
    
    public DriveCommand (Swerve swerve, BooleanSupplier xModeInput, DoubleSupplier strafeXAxis, DoubleSupplier strafeYAxis, DoubleSupplier rotateAxis) {
        this.swerve = swerve;
        this.xModeInput = xModeInput;
        this.strafeXAxis = strafeXAxis;
        this.strafeYAxis = strafeYAxis;
        this.rotateAxis = rotateAxis;
        addRequirements(swerve);
        
        RobotContainer.putConfigSendable("DriveCommand", this);
    }
    
    @Override
    public void initialize () {
        swerve.stop();
    }
    
    @Override
    public void execute () {
        if (xModeInput.getAsBoolean())
            swerve.xMode();
        else driveSwerveDefault();
    }
    
    private void driveSwerveDefault () {
        // Apply the input curves
        Input2D strafeInputRaw = new Input2D(strafeXAxis.getAsDouble(), strafeYAxis.getAsDouble());
        double rotateInputRaw = rotateAxis.getAsDouble();
        
        Input2D strafeSpeeds = InputCurve.apply(STRAFE_CURVE, strafeInputRaw).scale(5);
        double rotateSpeed = InputCurve.apply(ROTATE_CURVE, rotateInputRaw) * 6;
        
        DS_strafeX = strafeInputRaw.x();
        DS_strafeY = strafeInputRaw.y();
        DS_rotate = rotateInputRaw;
        
        // Robot orientation and ChassisSpeeds is based on the idea that +x is the front of the robot,
        // +y is the left side of the robot, etc.
        // Axes, of course, do not work like this
        swerve.moveRobotRelative(new ChassisSpeeds(-strafeSpeeds.y(), -strafeSpeeds.x(), rotateSpeed));
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("strafeX", () -> DS_strafeX, null);
        builder.addDoubleProperty("strafeY", () -> DS_strafeY, null);
        builder.addDoubleProperty("rotate", () -> DS_rotate, null);
    }
    
}
