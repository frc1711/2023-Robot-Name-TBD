package frc.robot.commands.auton;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Container;
import frc.robot.subsystems.swerve.Swerve;

public class DriveToTransform extends CommandBase {
    
    private final Swerve swerve;
    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(3, 0, 0),
        new PIDController(3, 0, 0),
        new ProfiledPIDController(7, 0, 0,
            new Constraints(4, 2)
        )
    );
    
    private final TrajectoryConfig config = new TrajectoryConfig(5, 1);
    
    private final Container<Transform2d> transformContainer;
    
    private Trajectory trajectory;
    private Rotation2d artificialRotation;
    private Supplier<Rotation2d> headerSupplier;
    
    private double startTime;
    
    public DriveToTransform (Swerve swerve, Container<Transform2d> transformContainer) {
        this.swerve = swerve;
        this.transformContainer = transformContainer;
        addRequirements(swerve);
        driveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(0.5)));
    }
    
    private double getSampleTime () {
        return WPIUtilJNI.getSystemTime() * 1.e-6 - startTime;
    }
    
    @Override
    public void initialize () {
        swerve.stop();
        
        startTime = WPIUtilJNI.getSystemTime() * 1.e-6;
        
        Pose2d startPose = swerve.getPose();
        Pose2d finalPose = startPose.plus(transformContainer.get());
        
        // The robot should follow the angle of the change from the start pose to final pose
        // So that it always faces in the same direction it drives in
        artificialRotation = finalPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        
        Rotation2d startRotation = startPose.getRotation();
        Rotation2d finalRotation = finalPose.getRotation();
        
        List<Pose2d> poses = List.of(
            new Pose2d(startPose.getTranslation(), artificialRotation),
            new Pose2d(finalPose.getTranslation(), artificialRotation)
        );
        
        trajectory = TrajectoryGenerator.generateTrajectory(
            poses,
            config
        );
        
        headerSupplier = () -> {
            double p = getSampleTime() / trajectory.getTotalTimeSeconds();
            return startRotation.interpolate(finalRotation, MathUtil.clamp(p, 0, 1));
        };
        
    }
    
    @Override
    public void execute () {
        Trajectory.State desiredPose = trajectory.sample(getSampleTime());
        
        ChassisSpeeds speeds = driveController.calculate(
            swerve.getPose(),
            desiredPose,
            headerSupplier.get()
        );
        
        swerve.moveRobotRelative(speeds);
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
    @Override
    public boolean isFinished () {
        return getSampleTime() > trajectory.getTotalTimeSeconds() && driveController.atReference();
    }
    
}

