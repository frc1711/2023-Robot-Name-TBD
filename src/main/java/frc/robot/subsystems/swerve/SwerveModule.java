package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

class SwerveModule {
    
    // TODO: Calculate this value
    private static final double METERS_PER_SEC_TO_DRIVE_VOLTS = 1;
    
    public static double getMaxDriveSpeedMetersPerSec () {
        return RobotController.getBatteryVoltage() / METERS_PER_SEC_TO_DRIVE_VOLTS;
    }
    
    private static CANSparkMax initializeMotor (int canId) {
        CANSparkMax motor = new CANSparkMax(canId, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
        return motor;
    }
    
    private final CANSparkMax driveMotor, steerMotor;
    private final ResettableEncoder steerEncoder;
    
    /**
     * 1 unit input for this PID controller is a full 360 deg rotation.
     * 1 unit output for this PID controller is one volt applied to the steer motor.
     */
    // TODO: Tune this PID
    private final PIDController steerPID = new PIDController(0.1, 0, 0);
    
    public SwerveModule (int driveSparkId, int steerSparkId, int steerCANCoderId) {
        driveMotor = initializeMotor(driveSparkId);
        steerMotor = initializeMotor(steerSparkId);
        
        steerEncoder = new ResettableEncoder(steerCANCoderId);
    }
    
    /**
     * Update this module's motor controllers to try to set the module to the desired speed and angle.
     * This method must be called periodically.
     * @param desiredState The desired {@link SwerveModuleState}.
     */
    public void update (SwerveModuleState desiredState) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, steerEncoder.getRotation());
        updateDriveMotor(optimizedDesiredState.speedMetersPerSecond);
        updateSteerMotor(optimizedDesiredState.angle);
    }
    
    private void updateDriveMotor (double desiredSpeedMetersPerSec) {
        double voltsOutput = METERS_PER_SEC_TO_DRIVE_VOLTS * desiredSpeedMetersPerSec;
        driveMotor.setVoltage(voltsOutput);
    }
    
    private void updateSteerMotor (Rotation2d desiredRotation) {
        // Measured in full, 360 degree rotations
        double measuredAngle = steerEncoder.getRotation().getRotations();
        double setAngle = desiredRotation.getRotations();
        
        // Steer PID calculates volts to apply to the steer motor
        double voltsOutput = steerPID.calculate(measuredAngle, setAngle);
        steerMotor.setVoltage(voltsOutput);
    }
    
    /**
     * Stop all motor controllers assigned to this {@link SwerveModule}.
     */
    public void stop () {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
    
}
