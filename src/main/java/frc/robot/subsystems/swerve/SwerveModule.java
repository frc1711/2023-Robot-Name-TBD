package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.api.CLAWLogger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

class SwerveModule {
    
    // TODO: Calculate this value
    private static final double
        METERS_PER_SEC_TO_DRIVE_VOLTS = 1;
    
    public static double getMaxDriveSpeedMetersPerSec () {
        return RobotController.getBatteryVoltage() / METERS_PER_SEC_TO_DRIVE_VOLTS;
    }
    
    private static CANSparkMax initializeMotor (int canId) {
        CANSparkMax motor = new CANSparkMax(canId, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
        return motor;
    }
    
    private final CLAWLogger log;
    private final CANSparkMax driveMotor, steerMotor;
    private final ResettableEncoder steerEncoder;
    private final SimpleMotorFeedforward steerFeedForward = new SimpleMotorFeedforward(0.12, 1);
    private double lastDriveVoltage = 0;
    
    /**
     * 1 unit input for this PID controller is a full 360 deg rotation.
     * 1 unit output for this PID controller is one volt applied to the steer motor.
     */
    private final RotationalPID steerPID;
    
    public SwerveModule (String modName, int driveSparkId, int steerSparkId, int steerCANCoderId) {
        log = CLAWLogger.getLogger("subsystems.swerve."+modName);
        steerPID = new RotationalPID("subsystems.swerve."+modName+".pid", 0.7/90, 0, 0, 6);
        
        driveMotor = initializeMotor(driveSparkId);
        steerMotor = initializeMotor(steerSparkId);
        
        steerEncoder = new ResettableEncoder("subsystems.swerve."+modName+".encoder", steerCANCoderId);
        steerEncoder.setInverted(true);
    }
    
    /**
     * Update this module's motor controllers to try to set the module to the desired speed and angle.
     * This method must be called periodically.
     * @param desiredState The desired {@link SwerveModuleState}.
     */
    public void update (SwerveModuleState desiredState) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getRotation());
        
        log.sublog("drive.Desired").out(desiredState.speedMetersPerSecond+" m/s");
        updateDriveMotor(optimizedDesiredState.speedMetersPerSecond);
        
        log.sublog("steer.Rotation").out(round(getRotation().getDegrees()) + " deg");
        log.sublog("steer.Desired").out(round(desiredState.angle.getDegrees()) + " deg");
        updateSteerMotor(optimizedDesiredState.angle);
    }
    
    private void updateDriveMotor (double desiredSpeedMetersPerSec) {
        double voltsOutput = METERS_PER_SEC_TO_DRIVE_VOLTS * desiredSpeedMetersPerSec;
        lastDriveVoltage = voltsOutput;
        driveMotor.setVoltage(voltsOutput);
        log.sublog("drive.Voltage").out(voltsOutput+" V");
    }
    
    private double round (double value) {
        return Math.round(value * 100) / 100.;
    }
    
    private void updateSteerMotor (Rotation2d desiredRotation) {
        double voltsOutput = steerFeedForward.calculate(steerPID.calculate(getRotation(), desiredRotation));
        
        log.sublog("steer.Voltage").out(voltsOutput + " V");
        steerMotor.setVoltage(voltsOutput);
    }
    
    /**
     * Stop all motor controllers assigned to this {@link SwerveModule}.
     */
    public void stop () {
        lastDriveVoltage = 0;
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
    
    /**
     * Zero the steer encoder and save its offset to the encoder.
     */
    public void zeroSteerEncoder () {
        steerEncoder.zeroRotation();
    }
    
    public double getDriveVoltage () {
        return lastDriveVoltage;
    }
    
    public Rotation2d getRotation () {
        return steerEncoder.getRotation();
    }
    
}
