// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import claw.CLAWRobot;
import claw.logs.CLAWLogger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
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
    
    public Swerve () {
        // Add configuration buttons to the shuffleboard
        RobotContainer.putConfigCommand("Zero Swerve Modules", new InstantCommand(() -> this.zeroModules(), this).ignoringDisable(true), true);
        RobotContainer.putConfigCommand("Zero Gyro", new InstantCommand(() -> this.zeroGyro(), this).ignoringDisable(true), true);
        
        // Add modules to the shuffleboard
        RobotContainer.putConfigSendable("fl-module", flModule);
        RobotContainer.putConfigSendable("fr-module", frModule);
        RobotContainer.putConfigSendable("rl-module", rlModule);
        RobotContainer.putConfigSendable("rr-module", rrModule);
        
        XboxController controller = new XboxController(0);
        
        LiveCommandTester<XboxController> tester = new LiveCommandTester<>(
            () -> controller,
            c -> {
                flModule.updateSteerMotor(c.getLeftX()*4);
                flModule.updateDriveMotor(0);
                
                frModule.updateSteerMotor(c.getLeftX()*4);
                frModule.updateDriveMotor(0);
                
                rlModule.updateSteerMotor(c.getLeftX()*4);
                rlModule.updateDriveMotor(0);
                
                rrModule.updateSteerMotor(c.getLeftX()*4);
                rrModule.updateDriveMotor(0);
                
                CLAWLogger.getLogger("test").out("hello there");
            },
            this::stop,
            this
        );
        
        RobotContainer.putConfigCommand("test command", tester.new TestCommand(), false);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.getMaxDriveSpeedMetersPerSec());
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
        // The gyro is oriented 90 degrees to the right, so to get the robot's pitch you actually need the gyro's roll
        return gyro.getRoll();
    }
    
    public double getRobotYaw () {
        // The gyro is oriented 90 degrees to the right, so to get the robot's pitch you actually need the gyro's roll
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
    
}
