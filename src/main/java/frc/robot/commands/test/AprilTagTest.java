package frc.robot.commands.test;

import java.util.Optional;

import claw.math.Transform;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight.Limelight.AprilTagData;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.VisionManager;

public class AprilTagTest extends CommandBase {
    
    private final Swerve swerve;
    private final Transform targetYawToTurnSpeed =
        Transform.clamp(-1, 1)
        .then(Transform.NEGATE)
        .then(x -> x*5)
        .then(new SlewRateLimiter(15, -15, 0)::calculate);
    
    public AprilTagTest (Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        swerve.stop();
    }
    
    private final LinearFilter filter = LinearFilter.movingAverage(20);
    
    @Override
    public void execute () {
        
        Optional<AprilTagData> aprilTag = VisionManager.getInstance().getArmAprilTag();
        if (aprilTag.isPresent()) {
            moveWith(aprilTag.get().targetPose().getX());
            
            double yaw = filter.calculate(aprilTag.get().targetPose().getRotation().toRotation2d().getDegrees());
            
            Rotation3d rot = aprilTag.get().targetPose().getRotation();
            double m = 180. / Math.PI;
            
            double relativeYaw = rot.getY();
            
            System.out.println(relativeYaw);
        } else {
            moveWith(0);
        }
    }
    
    private void moveWith (double x) {
        double turnSpeed = targetYawToTurnSpeed.apply(x);
        swerve.moveRobotRelative(new ChassisSpeeds(0, 0, turnSpeed));
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
    @Override
    public boolean isFinished () {
        return false;
    }
    
}
