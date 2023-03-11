package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.InputCurve.Input2D;
import frc.robot.commands.auton.SwerveTurnCorrector;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveTeleopAccelerationConstraints;

public class DriveCommand extends CommandBase {
    
    private static final InputCurve
        ROTATE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14),
        STRAFE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14);
    
    private final SwerveTeleopAccelerationConstraints accelerationConstraints = new SwerveTeleopAccelerationConstraints(14.7, 54);
    
    private final Swerve swerve;
    private final BooleanSupplier xModeInput, turboModeControl, resetGyro;
    private final DoubleSupplier strafeXAxis, strafeYAxis, rotateAxis;
    
    private final Supplier<Optional<Double>> lockInRotationControl;
    private final SwerveTurnCorrector turnCorrector = new SwerveTurnCorrector();
    
    private double DS_strafeX, DS_strafeY, DS_rotate;
    
    private Optional<Double> lockInRotation = Optional.empty();
    
    public DriveCommand (
            Swerve swerve,
            
            BooleanSupplier xModeInput,
            
            DoubleSupplier strafeXAxis,
            DoubleSupplier strafeYAxis,
            DoubleSupplier rotateAxis,
            
            BooleanSupplier turboModeControl,
            BooleanSupplier resetGyro,
            
            Supplier<Optional<Double>>  lockInRotationControl
        ) {
        this.swerve = swerve;
        this.xModeInput = xModeInput;
        
        this.strafeXAxis = strafeXAxis;
        this.strafeYAxis = strafeYAxis;
        this.rotateAxis = rotateAxis;
        
        this.turboModeControl = turboModeControl;
        this.resetGyro = resetGyro;
        
        this.lockInRotationControl = lockInRotationControl;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        // Stop swerve drive so it doesn't move on startup
        swerve.stop();
        
        // Reset acceleration constraints to the current timestamp
        accelerationConstraints.reset();
    }
    
    @Override
    public void execute () {
        // Driving swerve
        if (xModeInput.getAsBoolean()) {
            // Put in x mode
            swerve.xMode();
            
            // Turn off the lock-in rotation so it doesn't go right back to lock-in after pressing x-mode
            lockInRotation = Optional.empty();
        } else {
            driveSwerveDefault();
        }
        
        // Zeroing gyro
        if (resetGyro.getAsBoolean()) {
            swerve.zeroGyroTeleop();
        }
    }
    
    private double getTurnSpeed (double userSuppliedSpeed) {
        // Set the lockInRotation field depending on given user input
        Optional<Double> givenLockInRotation = lockInRotationControl.get();
        if (givenLockInRotation.isPresent()) {
            lockInRotation = givenLockInRotation;
        } else if (userSuppliedSpeed != 0) {
            lockInRotation = Optional.empty();
        }
        
        // Return a turn speed according to the presense (or absense) of a lock-in rotation
        if (lockInRotation.isPresent()) {
            // If there is an active lock-in rotation, then turn accordingly
            return turnCorrector.getCorrectionSpeed(swerve.getTeleopDriveRobotRotation(), Rotation2d.fromDegrees(-lockInRotation.get()));
        } else {
            // Otherwise, turn according to user input
            return userSuppliedSpeed * 6.5;
        }
    }
    
    private void driveSwerveDefault () {
        // Get the raw input
        Input2D strafeInputRaw = new Input2D(strafeXAxis.getAsDouble(), strafeYAxis.getAsDouble());
        double rotateInputRaw = rotateAxis.getAsDouble();
        
        // Apply input curves
        Input2D strafeSpeeds = InputCurve.apply(STRAFE_CURVE, strafeInputRaw).scale(4);
        double rotateSpeed = getTurnSpeed(-InputCurve.apply(ROTATE_CURVE, rotateInputRaw));
        
        // Handle turbo mode for strafe speeds
        if (turboModeControl.getAsBoolean()) {
            strafeSpeeds = strafeSpeeds.scale(3);
        }
        
        // Update driverstation dashboard fields
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
        // Stop swerve drive so it doesn't continue driving
        swerve.stop();
        
        // Do not lock in rotation when command resumes
        lockInRotation = Optional.empty();
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("strafeX", () -> DS_strafeX, null);
        builder.addDoubleProperty("strafeY", () -> DS_strafeY, null);
        builder.addDoubleProperty("rotate", () -> DS_rotate, null);
    }
    
}
