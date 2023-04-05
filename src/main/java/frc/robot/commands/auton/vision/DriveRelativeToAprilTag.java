package frc.robot.commands.auton.vision;

import frc.robot.commands.Container;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveToTransform;
import frc.robot.subsystems.swerve.Swerve;

public class DriveRelativeToAprilTag extends SequentialCommandGroup {
    
    private final Container<Transform2d>
        aprilTagTransform = new Container<>(),
        currentPoseToFinal = new Container<>();
    
    public DriveRelativeToAprilTag (Swerve swerve, Transform2d tagToFinalPose) {
        addCommands(
            new InstantCommand(() -> aprilTagTransform.set(null)),
            new GetAprilTag(aprilTagTransform),
            new InstantCommand(() -> {
                
                // Get the transform from the current pose to the final pose
                currentPoseToFinal.set(aprilTagTransform.get().plus(tagToFinalPose));
                
            }),
            new DriveToTransform(swerve, currentPoseToFinal)
        );
    }
    
}
