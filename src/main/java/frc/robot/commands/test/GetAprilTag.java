package frc.robot.commands.test;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight.Limelight.AprilTagData;
import frc.robot.vision.VisionManager;

public class GetAprilTag extends CommandBase {
    
    private final Container<Transform2d> data;
    
    public GetAprilTag (Container<Transform2d> data) {
        this.data = data;
    }
    
    @Override
    public void execute () {
        Optional<AprilTagData> optData = VisionManager.getInstance().getArmAprilTag();
        
        if (optData.isPresent()) {
            AprilTagData tag = optData.get();
            
            double tagX = tag.targetPose().getZ();
            double tagY = -tag.targetPose().getX();
            double tagYawRadians = tag.targetPose().getRotation().getY();
            
            data.set(new Transform2d(
                new Translation2d(tagX, tagY),
                Rotation2d.fromDegrees(180).minus(new Rotation2d(tagYawRadians))
            ));
            
        }
        
    }
    
    @Override
    public boolean isFinished () {
        return data.get() != null;
    }
    
}
