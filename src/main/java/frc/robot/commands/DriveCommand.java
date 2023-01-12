package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class DriveCommand extends CommandBase {
    
    private final Swerve swerve;
    private final DoubleSupplier strafeXAxis, strafeYAxis, rotateAxis;
    
    public DriveCommand (Swerve swerve, DoubleSupplier strafeXAxis, DoubleSupplier strafeYAxis, DoubleSupplier rotateAxis) {
        this.swerve = swerve;
        this.strafeXAxis = strafeXAxis;
        this.strafeYAxis = strafeYAxis;
        this.rotateAxis = rotateAxis;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        swerve.stop();
    }
    
    @Override
    public void execute () {
        // TODO: Tuning this will also involve implementing CLAW HID interfaces (as they are being created)
        
        // Robot orientation and ChassisSpeeds is based on the idea that +x is the front of the robot,
        // +y is the left side of the robot, etc.
        // Axes, of course, do not work like this
        double xSpeed = -strafeYAxis.getAsDouble();
        double ySpeed = strafeXAxis.getAsDouble();
        double rotateSpeed = rotateAxis.getAsDouble();
        
        // Perform field-relative robot movement
        swerve.moveFieldRelative(new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed));
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
}
