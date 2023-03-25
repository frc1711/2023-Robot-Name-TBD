package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class AprilTagTest extends SequentialCommandGroup {
    
    private final Container<Transform2d> aprilTagTransform = new Container<Transform2d>(null);
    
    public AprilTagTest (Swerve swerve) {
        addCommands(
            new DriveToTransform(swerve, new Container<Transform2d>(
                new Transform2d(
                    new Translation2d(-0.5, 0),
                    Rotation2d.fromDegrees(0)
                )
            ))
            // new GetAprilTag(aprilTagTransform),
            // new InstantCommand(() -> {
                
            //     // Transform from tag to final pose
            //     Transform2d tagToFinalPose = new Transform2d(
            //         new Translation2d(1.5, 0),
            //         Rotation2d.fromDegrees(180)
            //     );
                
            //     // Get the transform from the current pose to the final pose
            //     Transform2d currentPoseToFinal = aprilTagTransform.get().plus(tagToFinalPose);
                
            //     System.out.println("April tag transform:   " + aprilTagTransform.get());
            //     System.out.println("Tag to final pose:     " + tagToFinalPose);
            //     System.out.println("Current pose to final: " + currentPoseToFinal);
                
            // })
        );
    }
    
}
