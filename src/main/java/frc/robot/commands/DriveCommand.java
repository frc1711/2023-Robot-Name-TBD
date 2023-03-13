package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.InputCurve.Input2D;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveTeleopAccelerationConstraints;

public class DriveCommand extends CommandBase {
    
    private static final InputCurve
        ROTATE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14),
        STRAFE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14);
    
    private final SwerveTeleopAccelerationConstraints accelerationConstraints = new SwerveTeleopAccelerationConstraints(14, 32);
    
    private final Swerve swerve;
    private final BooleanSupplier xModeInput, turboModeControl, resetGyro;
    private final DoubleSupplier strafeXAxis, strafeYAxis, rotateAxis;
    
    private double DS_strafeX, DS_strafeY, DS_rotate;
    
    public DriveCommand (
            Swerve swerve,
            
            BooleanSupplier xModeInput,
            
            DoubleSupplier strafeXAxis,
            DoubleSupplier strafeYAxis,
            DoubleSupplier rotateAxis,
            
            BooleanSupplier turboModeControl,
            BooleanSupplier resetGyro
        ) {
        this.swerve = swerve;
        this.xModeInput = xModeInput;
        
        this.strafeXAxis = strafeXAxis;
        this.strafeYAxis = strafeYAxis;
        this.rotateAxis = rotateAxis;
        
        this.turboModeControl = turboModeControl;
        this.resetGyro = resetGyro;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        swerve.stop();
        accelerationConstraints.reset();
    }
    
    @Override
    public void execute () {
        // Driving swerve
        if (xModeInput.getAsBoolean())
            swerve.xMode();
        else driveSwerveDefault();
        
        // Zeroing gyro
        if (resetGyro.getAsBoolean()) {
            swerve.zeroGyroTeleop();
        }
    }
    
    private void driveSwerveDefault () {
        // Apply the input curves
        Input2D strafeInputRaw = new Input2D(strafeXAxis.getAsDouble(), strafeYAxis.getAsDouble());
        double rotateInputRaw = rotateAxis.getAsDouble();
        
        Input2D strafeSpeeds = InputCurve.apply(STRAFE_CURVE, strafeInputRaw).scale(4);
        if (turboModeControl.getAsBoolean()) {
            strafeSpeeds = strafeSpeeds.scale(3);
        }
        double rotateSpeed = -InputCurve.apply(ROTATE_CURVE, rotateInputRaw) * 5;
        
        DS_strafeX = strafeInputRaw.x();
        DS_strafeY = strafeInputRaw.y();
        DS_rotate = rotateInputRaw;
        
        // Robot orientation and ChassisSpeeds is based on the idea that +x is the front of the robot,
        // +y is the left side of the robot, etc.
        // Axes, of course, do not work like this
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(-strafeSpeeds.y(), -strafeSpeeds.x(), rotateSpeed);
        
        // Apply the final acceleration constraints to the desired speeds
        swerve.moveFieldRelativeTeleop(accelerationConstraints.applyToSpeeds(desiredSpeeds));
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
