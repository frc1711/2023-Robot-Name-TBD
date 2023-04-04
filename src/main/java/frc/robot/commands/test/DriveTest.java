package frc.robot.commands.test;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class DriveTest extends CommandBase {
    
    private final Swerve swerve;
    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(5, 0, 0,
            new Constraints(4, 2)
        )
    );
    
    private final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(1, -1, Rotation2d.fromDegrees(45)),
            new Pose2d(1, 1, Rotation2d.fromDegrees(90)),
            new Pose2d(0, 1, Rotation2d.fromDegrees(180)),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0))
        ),
        new TrajectoryConfig(5, 1)
    );
    
    private double time;
    
    public DriveTest (Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        swerve.stop();
        time = WPIUtilJNI.getSystemTime() * 1.e-6;
    }
    
    @Override
    public void execute () {
        double sysTime = WPIUtilJNI.getSystemTime() * 1.e-6 - time;
        
        Trajectory.State desiredPose = trajectory.sample(sysTime);
        ChassisSpeeds speeds = driveController.calculate(swerve.getPose(), desiredPose, desiredPose.poseMeters.getRotation());
        // System.out.println(speeds);
        swerve.moveRobotRelative(speeds);
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
}
