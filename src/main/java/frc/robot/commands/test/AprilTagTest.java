package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class AprilTagTest extends SequentialCommandGroup {
    
    private final Container<Transform2d>
        aprilTagTransform = new Container<>(),
        currentPoseToFinal = new Container<>();
    
    public AprilTagTest (Swerve swerve) {
        addCommands(
            new GetAprilTag(aprilTagTransform),
            new InstantCommand(() -> {
                
                // Transform from tag to final pose
                Transform2d tagToFinalPose = new Transform2d(
                    new Translation2d(Units.inchesToMeters(26), 0),
                    Rotation2d.fromDegrees(180)
                );
                
                // Get the transform from the current pose to the final pose
                currentPoseToFinal.set(aprilTagTransform.get().plus(tagToFinalPose));
                
                System.out.println("April tag transform:   " + aprilTagTransform);
                System.out.println("Tag to final pose:     " + tagToFinalPose);
                System.out.println("Current pose to final: " + currentPoseToFinal);
                
            }),
            new DriveToTransform(swerve, currentPoseToFinal)
        );
    }
    
}
