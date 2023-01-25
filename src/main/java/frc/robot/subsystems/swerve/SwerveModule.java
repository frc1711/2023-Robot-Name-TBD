package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWLogger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

class SwerveModule {
    
    // TODO: Calculate this value
    private static final double
        METERS_PER_SEC_TO_DRIVE_VOLTS = 1;
    
    private static final SimpleMotorFeedforward STEER_FEEDFORWARD = new SimpleMotorFeedforward(0.14, 1);
    private static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.2, 1);
    
    public static double getMaxDriveSpeedMetersPerSec () {
        return RobotController.getBatteryVoltage() / METERS_PER_SEC_TO_DRIVE_VOLTS;
    }
    
    private static CANSparkMax initializeMotor (int canId) {
        CANSparkMax motor = new CANSparkMax(canId, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        return motor;
    }
    
    private final CLAWLogger log;
    private final CANSparkMax driveMotor, steerMotor;
    private final ResettableEncoder steerEncoder;
    private double lastDriveVoltage = 0;
    
    /**
     * 1 unit input for this PID controller is a full 360 deg rotation.
     * 1 unit output for this PID controller is one volt applied to the steer motor.
     */
    private final RotationalPID steerPID;
    
    public SwerveModule (String modName, int driveSparkId, int steerSparkId, int steerCANCoderId) {
        log = CLAWLogger.getLogger("subsystems.swerve."+modName);
        steerPID = new RotationalPID("subsystems.swerve."+modName+".pid", 6/90., 0, 0, 6);
        
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
        
        if (desiredState.speedMetersPerSecond != 0) {
            log.sublog("steer.Rotation").out(round(getRotation().getDegrees()) + " deg");
            log.sublog("steer.Desired").out(round(desiredState.angle.getDegrees()) + " deg");
            updateSteerMotor(optimizedDesiredState.angle);
        } else {
            steerMotor.setVoltage(0);
        }
    }
    
    public void xMode (boolean turnDir) {
        SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(45 + (turnDir ? 90 : 0)));
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getRotation());
        updateDriveMotor(0);
        updateSteerMotor(optimizedState.angle);
    }
    
    private void updateDriveMotor (double desiredSpeedMetersPerSec) {
        double voltsOutput = DRIVE_FEEDFORWARD.calculate(METERS_PER_SEC_TO_DRIVE_VOLTS * desiredSpeedMetersPerSec);
        lastDriveVoltage = voltsOutput;
        driveMotor.setVoltage(voltsOutput);
        log.sublog("drive.Voltage").out(voltsOutput+" V");
    }
    
    private double round (double value) {
        return Math.round(value * 100) / 100.;
    }
    
    private void updateSteerMotor (Rotation2d desiredRotation) {
        double voltsOutput = STEER_FEEDFORWARD.calculate(steerPID.calculate(getRotation(), desiredRotation));
        
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
