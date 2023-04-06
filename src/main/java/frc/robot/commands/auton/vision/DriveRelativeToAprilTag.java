package frc.robot.commands.auton.vision;

import frc.robot.commands.Container;
import frc.robot.commands.auton.base.DriveToPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class DriveRelativeToAprilTag extends SequentialCommandGroup {
    
    private final Container<Transform2d>
        aprilTagTransform = new Container<>(),
        currentPoseToFinal = new Container<>();
    
    private static final Transform2d ADJUST_FOR_LIMELIGHT_INACCURACY = new Transform2d(
        new Translation2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(3)
        ),
        Rotation2d.fromDegrees(182.5)
    );
    
    public DriveRelativeToAprilTag (Swerve swerve, Transform2d finalPoseToTag) {
        
        Transform2d finalPoseToTagAdj = finalPoseToTag.plus(ADJUST_FOR_LIMELIGHT_INACCURACY);
        
        addCommands(
            new InstantCommand(() -> aprilTagTransform.set(null)),
            new GetAprilTag(aprilTagTransform),
            new InstantCommand(() -> {
                
                // Get the transform from the current pose to the final pose
                currentPoseToFinal.set(aprilTagTransform.get().plus(finalPoseToTagAdj.inverse()));
                
            }),
            new ProxyCommand(() -> new DriveToPosition(swerve, currentPoseToFinal.get()))
        );
    }
    
    public static DriveRelativeToAprilTag getHighCentering (Swerve swerve) {
        return new DriveRelativeToAprilTag(
            swerve,
            
            // Note: This transform isn't exactly precise, because the april tag in our room is slightly off-center.
            // Adjust this as needed.
            new Transform2d(
                new Translation2d(
                    Units.inchesToMeters(28),
                    Units.inchesToMeters(0)
                ),
                Rotation2d.fromDegrees(0)
            )
        );
    }
    
    public static DriveRelativeToAprilTag getMidCentering (Swerve swerve) {
        return new DriveRelativeToAprilTag(
            swerve,
            
            // Note: This transform isn't exactly precise, because the april tag in our room is slightly off-center.
            // Adjust this as needed.
            new Transform2d(
                new Translation2d(
                    Units.inchesToMeters(42),
                    Units.inchesToMeters(0)
                ),
                Rotation2d.fromDegrees(0)
            )
        );
    }
    
}
