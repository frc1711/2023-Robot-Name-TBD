// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import claw.CLAWRobot;
import claw.Setting;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(0.5, 0.5),
        FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(0.5, -0.5),
        REAR_LEFT_MODULE_TRANSLATION = new Translation2d(-0.5, 0.5),
        REAR_RIGHT_MODULE_TRANSLATION = new Translation2d(-0.5, -0.5);
    
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
    
    public Swerve () {
        RobotContainer.putConfigSendable("Swerve Subsystem", this);
        
        // Add configuration buttons to the shuffleboard
        RobotContainer.putConfigCommand("Zero Swerve Modules", new InstantCommand(() -> this.zeroModules(), this).ignoringDisable(true), true);
        RobotContainer.putConfigCommand("Zero Gyro", new InstantCommand(() -> this.zeroGyro(), this).ignoringDisable(true), true);
        
        // Add modules to the shuffleboard
        RobotContainer.putConfigSendable("fl-module", flModule);
        RobotContainer.putConfigSendable("fr-module", frModule);
        RobotContainer.putConfigSendable("rl-module", rlModule);
        RobotContainer.putConfigSendable("rr-module", rrModule);
        
        LiveCommandTester tester = new LiveCommandTester(
            "No special usage. Fully automatic.",
            liveValues -> {
                
                double angle = System.currentTimeMillis()/5. % 360.;
                
                liveValues.setField("Angle", angle + " deg");
                
                flModule.updateSteerMotor(Rotation2d.fromDegrees(angle));
                flModule.updateDriveMotor(0);
                liveValues.setField("fl", flModule.getRotation().getDegrees());
                
                frModule.updateSteerMotor(Rotation2d.fromDegrees(angle));
                frModule.updateDriveMotor(0);
                liveValues.setField("fr", frModule.getRotation().getDegrees());
                
                rlModule.updateSteerMotor(Rotation2d.fromDegrees(angle));
                rlModule.updateDriveMotor(0);
                liveValues.setField("rl", rlModule.getRotation().getDegrees());
                
                rrModule.updateSteerMotor(Rotation2d.fromDegrees(angle));
                rrModule.updateDriveMotor(0);
                liveValues.setField("rr", rrModule.getRotation().getDegrees());
                
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
        flModule.update(moduleStates[0]);
        frModule.update(moduleStates[1]);
        rlModule.update(moduleStates[2]);
        rrModule.update(moduleStates[3]);
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
        return gyro.getPitch();
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
        builder.addDoubleProperty("Yaw", gyro::getYaw, null);
    }
    
}
