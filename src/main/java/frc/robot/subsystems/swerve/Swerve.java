// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import claw.CLAWRobot;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.IDMap;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;
import frc.robot.limelight.Limelight;
import frc.robot.limelight.Limelight.AprilTagData;

public class Swerve extends SubsystemBase {
    
    private static Swerve swerve;
    
    public static Swerve getInstance () {
        if (swerve == null)
            swerve = new Swerve();
        return swerve;
    }
    
    private static final Translation2d
        FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(0.404, 0.404),
        FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(0.404, -0.404),
        REAR_LEFT_MODULE_TRANSLATION = new Translation2d(-0.404, 0.404),
        REAR_RIGHT_MODULE_TRANSLATION = new Translation2d(-0.404, -0.404);
    
    private final AHRS gyro = new AHRS();
    
    private final SwerveModule
        flModule = new SwerveModule(
            IDMap.FRONT_LEFT_MODULE_DRIVE_SPARK_ID,
            IDMap.FRONT_LEFT_MODULE_STEER_SPARK_ID,
            IDMap.FRONT_LEFT_MODULE_STEER_CANCODER_ID),
        frModule = new SwerveModule(
            IDMap.FRONT_RIGHT_MODULE_DRIVE_SPARK_ID,
            IDMap.FRONT_RIGHT_MODULE_STEER_SPARK_ID,
            IDMap.FRONT_RIGHT_MODULE_STEER_CANCODER_ID),
        rlModule = new SwerveModule(
            IDMap.REAR_LEFT_MODULE_DRIVE_SPARK_ID,
            IDMap.REAR_LEFT_MODULE_STEER_SPARK_ID,
            IDMap.REAR_LEFT_MODULE_STEER_CANCODER_ID),
        rrModule = new SwerveModule(
            IDMap.REAR_RIGHT_MODULE_DRIVE_SPARK_ID,
            IDMap.REAR_RIGHT_MODULE_STEER_SPARK_ID,
            IDMap.REAR_RIGHT_MODULE_STEER_CANCODER_ID);
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        FRONT_LEFT_MODULE_TRANSLATION,
        FRONT_RIGHT_MODULE_TRANSLATION,
        REAR_LEFT_MODULE_TRANSLATION,
        REAR_RIGHT_MODULE_TRANSLATION
    );
    
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[]{
            flModule.getPosition(),
            frModule.getPosition(),
            rlModule.getPosition(),
            rrModule.getPosition(),
        },
        new Pose2d()
    );
    
    private final Constraints robotConstraints = new Constraints(8, 8);
    
    private final PIDController
        autonDriveXController = new PIDController(1, 0, 0),
        autonDriveYController = new PIDController(1, 0, 0);
    
    private final ProfiledPIDController autonDriveThetaController = new ProfiledPIDController(
        1,
        0,
        0,
        robotConstraints
    );
    
    private final Field2d sendableField = new Field2d();
    
    private double measurementOffset = 0;
    
    public Swerve () {
        RobotContainer.putConfigSendable("Swerve Subsystem", this);
        RobotContainer.putConfigSendable("Position", sendableField);
        
        // Add configuration buttons to the shuffleboard
        RobotContainer.putConfigCommand("Zero Swerve Modules", new InstantCommand(() -> this.zeroModules(), this).ignoringDisable(true), true);
        RobotContainer.putConfigCommand("Zero Gyro", new InstantCommand(() -> this.zeroGyro(), this).ignoringDisable(true), true);
        
        // Add modules to the shuffleboard
        RobotContainer.putConfigSendable("fl-module", flModule);
        RobotContainer.putConfigSendable("fr-module", frModule);
        RobotContainer.putConfigSendable("rl-module", rlModule);
        RobotContainer.putConfigSendable("rr-module", rrModule);
        
        XboxController controller = new XboxController(3);
        
        LiveCommandTester tester = new LiveCommandTester(
            "No special usage. Fully automatic.",
            liveValues -> {
                
                SwerveModuleState desiredState;
                double measurement = 0;
                // double measurement =
                //     (flModule.getDisplacementMeters() +
                //     frModule.getDisplacementMeters() +
                //     rlModule.getDisplacementMeters() +
                //     rrModule.getDisplacementMeters()) / 4;
                
                if (controller.getAButton()) {
                    desiredState = new SwerveModuleState(1, Rotation2d.fromDegrees(0));
                    liveValues.setField("measurement", (measurement - measurementOffset) + " m");
                } else {
                    desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
                    measurementOffset = measurement;
                }
                
                flModule.update(desiredState);
                frModule.update(desiredState);
                rlModule.update(desiredState);
                rrModule.update(desiredState);
                
            },
            this::stop,
            this
        );
        
        CLAWRobot.getExtensibleCommandInterpreter().addCommandProcessor(
            tester.toCommandProcessor("swervetest")
        );
    }
    
    /**
     * Update all the swerve drive motor controllers to try to match the given robot-relative {@link ChassisSpeeds}.
     * This method must be called periodically.
     * @param speeds The {@code ChassisSpeeds} to try to match with the swerve drive.
     */
    public void moveRobotRelative (ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveModule.desaturateWheelSpeeds(moduleStates);
    }
    
    private void setModuleStates (SwerveModuleState[] states) {
        flModule.update(states[0]);
        frModule.update(states[1]);
        rlModule.update(states[2]);
        rrModule.update(states[3]);
    }
    
    /**
     * Update all the swerve drive motor controllers to try to match the given field-relative {@link ChassisSpeeds}.
     * This method must be called periodically.
     * @param speeds The {@code ChassisSpeeds} to try to match with the swerve drive.
     */
    public void moveFieldRelative (ChassisSpeeds speeds) {
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
        moveRobotRelative(robotRelativeSpeeds);
    }
    
    public void xMode () {
        flModule.xMode(false);
        frModule.xMode(true);
        rlModule.xMode(true);
        rrModule.xMode(false);
    }
    
    /**
     * Zero all steer encoders and save their offsets.
     */
    public void zeroModules () {
        flModule.zeroSteerEncoder();
        frModule.zeroSteerEncoder();
        rlModule.zeroSteerEncoder();
        rrModule.zeroSteerEncoder();
    }
    
    /**
     * Zeroes the gyro's yaw so that field-relative robot driving will see the current robot position as the
     * "starting orientation".
     */
    public void zeroGyro () {
        gyro.zeroYaw();
    }
    
    public double getRobotPitch () {
        return gyro.getRoll();
    }
    
    public double getRobotYaw () {
        return gyro.getYaw();
    }
    
    /**
     * Stop all swerve modules immediately.
     */
    public void stop () {
        flModule.stop();
        frModule.stop();
        rlModule.stop();
        rrModule.stop();
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("Yaw", this::getRobotYaw, null);
        builder.addDoubleProperty("Pitch", this::getRobotPitch, null);
    }
    
    public Command getControllerCommand (Trajectory trajectory) {
        return new SwerveControllerCommand(
            trajectory,
            poseEstimator::getEstimatedPosition,
            kinematics,
            new HolonomicDriveController(
                autonDriveXController,
                autonDriveYController,
                autonDriveThetaController
            ),
            this::setModuleStates,
            this
        );
    }
    
    @Override
    public void periodic () {
        poseEstimator.update(gyro.getRotation2d(), new SwerveModulePosition[]{
            flModule.getPosition(),
            frModule.getPosition(),
            rlModule.getPosition(),
            rrModule.getPosition(),
        });
        
        Optional<AprilTagData> tag = Limelight.getAprilTag();
        if (tag.isPresent()) {
            // TODO: make more robust to bad vision data, figure out why we're not seeing the apriltag in the limelight NT API
            AprilTagData data = tag.get();
            double time = Timer.getFPGATimestamp();
            poseEstimator.addVisionMeasurement(data.robotPose().toPose2d(), time);
        }
        
        sendableField.setRobotPose(poseEstimator.getEstimatedPosition());
    }
    
}
