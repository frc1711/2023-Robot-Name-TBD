package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class DriveToTag extends SequentialCommandGroup {
    
    private final Container<Pose2d> driveToPose = new Container<Pose2d>(null);
    
    public DriveToTag (Swerve swerve) {
        
    }
    
}
