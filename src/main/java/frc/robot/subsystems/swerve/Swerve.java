// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import claw.CLAWRobot;
import claw.Setting;
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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.IDMap;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;

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
            new Setting<>("FL_MODULE_ENCODER_OFFSET", ()->0.),
            IDMap.FRONT_LEFT_MODULE_DRIVE_SPARK_ID,
            IDMap.FRONT_LEFT_MODULE_STEER_SPARK_ID,
            IDMap.FRONT_LEFT_MODULE_STEER_CANCODER_ID),
        frModule = new SwerveModule(
            new Setting<>("FR_MODULE_ENCODER_OFFSET", ()->0.),
            IDMap.FRONT_RIGHT_MODULE_DRIVE_SPARK_ID,
            IDMap.FRONT_RIGHT_MODULE_STEER_SPARK_ID,
            IDMap.FRONT_RIGHT_MODULE_STEER_CANCODER_ID),
        rlModule = new SwerveModule(
            new Setting<>("RL_MODULE_ENCODER_OFFSET", ()->0.),
            IDMap.REAR_LEFT_MODULE_DRIVE_SPARK_ID,
            IDMap.REAR_LEFT_MODULE_STEER_SPARK_ID,
            IDMap.REAR_LEFT_MODULE_STEER_CANCODER_ID),
        rrModule = new SwerveModule(
            new Setting<>("RR_MODULE_ENCODER_OFFSET", ()->0.),
            IDMap.REAR_RIGHT_MODULE_DRIVE_SPARK_ID,
            IDMap.REAR_RIGHT_MODULE_STEER_SPARK_ID,
            IDMap.REAR_RIGHT_MODULE_STEER_CANCODER_ID);
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        FRONT_LEFT_MODULE_TRANSLATION,
        FRONT_RIGHT_MODULE_TRANSLATION,
        REAR_LEFT_MODULE_TRANSLATION,
        REAR_RIGHT_MODULE_TRANSLATION
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
    
    private Rotation2d absoluteRobotRotationOffset = gyro.getRotation2d();
    private Rotation2d gyroTeleopYawOffset = Rotation2d.fromDegrees(0);
    private double gyroZeroPitchOffset = 0;
    
    
    private double measurementOffset = 0;
    
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getRobotRotation(),
        new SwerveModulePosition[]{
            flModule.getPosition(),
            frModule.getPosition(),
            rlModule.getPosition(),
            rrModule.getPosition(),
        },
        new Pose2d(0, 0, getRobotRotation())
    );
    
    private Swerve () {
        RobotContainer.putConfigSendable("Swerve Subsystem", this);
        RobotContainer.putConfigSendable("Position", sendableField);
        
        // Add configuration buttons to the shuffleboard
        RobotContainer.putConfigCommand("Zero Swerve Modules", new InstantCommand(this::zeroModules, this).ignoringDisable(true), true);
        RobotContainer.putConfigCommand("Teleop Zero Gyro", new InstantCommand(this::zeroGyroTeleop, this).ignoringDisable(true), true);
        
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
        
        gyroZeroPitchOffset = getRobotPitchRaw();
    }
    
    /**
     * Update all the swerve drive motor controllers to try to match the given robot-relative {@link ChassisSpeeds}.
     * This method must be called periodically.
     * @param speeds The {@code ChassisSpeeds} to try to match with the swerve drive.
     */
    public void moveRobotRelative (ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveModule.desaturateWheelSpeeds(moduleStates);
        setModuleStates(moduleStates);
    }
    
    private void setModuleStates (SwerveModuleState[] states) {
        flModule.update(states[0]);
        frModule.update(states[1]);
        rlModule.update(states[2]);
        rrModule.update(states[3]);
    }
    
    /**
     * Update all the swerve drive motor controllers to try to match the given field-relative {@link ChassisSpeeds}.
     * This method must be called periodically. The movement will be relative to the last zeroGyroTeleop reset.
     * @param speeds The {@code ChassisSpeeds} to try to match with the swerve drive.
     */
    public ChassisSpeeds getFieldRelativeTeleopChassisSpeeds (ChassisSpeeds speeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            getTeleopDriveRobotRotation()
        );
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
    public void zeroGyroTeleop () {
        gyroTeleopYawOffset = getRobotRotation();
    }
    
    public double getRobotPitch () {
        return getRobotPitchRaw() - gyroZeroPitchOffset;
    }
    
    private double getRobotPitchRaw () {
        return gyro.getRoll();
    }
    
    public Rotation2d getTeleopDriveRobotRotation () {
        return getRobotRotation().minus(gyroTeleopYawOffset);
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
        builder.addDoubleProperty("Absolute Yaw", () -> getRobotRotation().getDegrees(), null);
        builder.addDoubleProperty("Pitch", this::getRobotPitch, null);
        builder.addDoubleProperty("Teleop Yaw", () -> {
            return getTeleopDriveRobotRotation().getDegrees();
        }, null);
    }
    
    public Command getControllerCommand (Pose2d... waypoints) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(waypoints), new TrajectoryConfig(4, 4));
        
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
    
    public Pose2d getPose () {
        return poseEstimator.getEstimatedPosition();
    }
    
    public Rotation2d getRobotRotation () {
        // P = R - A
        return gyro.getRotation2d().minus(absoluteRobotRotationOffset);
    }
    
    /**
     * USE WITH CAUTION - THIS ADJUSTS ROTATION AND POSE READINGS
     */
    public void setPose (Pose2d newPose) {
        // A = R - P
        // Adjust absoluteRobotRotationOffset to reset the getRobotRotation reading to fit the pose
        absoluteRobotRotationOffset = gyro.getRotation2d().minus(newPose.getRotation());
        
        poseEstimator.resetPosition(getRobotRotation(), new SwerveModulePosition[]{
            flModule.getPosition(),
            frModule.getPosition(),
            rlModule.getPosition(),
            rrModule.getPosition(),
        }, newPose);
    }
    
    @Override
    public void periodic () {
        poseEstimator.update(getRobotRotation(), new SwerveModulePosition[]{
            flModule.getPosition(),
            frModule.getPosition(),
            rlModule.getPosition(),
            rrModule.getPosition(),
        });
        
        sendableField.setRobotPose(poseEstimator.getEstimatedPosition());
    }
    
}
