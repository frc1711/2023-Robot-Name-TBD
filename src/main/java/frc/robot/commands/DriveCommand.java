package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
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
        // TODO: Tuning these speeds will also involve implementing CLAW HID interfaces (as they are being created)
        
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
