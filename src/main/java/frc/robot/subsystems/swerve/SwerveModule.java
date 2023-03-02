package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;

class SwerveModule implements Sendable {
    
    // TODO: Calculate this value
    private static final double
        METERS_PER_SEC_TO_DRIVE_VOLTS = 1;
    
    private static final SimpleMotorFeedforward STEER_FEEDFORWARD = new SimpleMotorFeedforward(0.14, 1);
    private static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.2, 1);
    
    private static double getMaxDriveSpeedMetersPerSec () {
        return RobotController.getBatteryVoltage() / METERS_PER_SEC_TO_DRIVE_VOLTS;
    }
    
    public static void desaturateWheelSpeeds (SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, getMaxDriveSpeedMetersPerSec());
    }
    
    private static CANSparkMax initializeMotor (int canId) {
        CANSparkMax motor = new CANSparkMax(canId, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        return motor;
    }
    
    private final CANSparkMax driveMotor, steerMotor;
    private final ResettableEncoder steerEncoder;
    
    private double
        DS_steerOutputVoltage = 0,
        DS_driveOutputVoltage = 0,
        DS_unoptimizedDesiredRotation = 0,
        DS_desiredRotation = 0,
        DS_desiredDriveSpeed = 0;
    
    private boolean DS_driveEnabled = true;
    
    /**
     * 1 unit input for this PID controller is a full 360 deg rotation.
     * 1 unit output for this PID controller is one volt applied to the steer motor.
     */
    private final RotationalPID steerPID;
    
    public SwerveModule (int driveSparkId, int steerSparkId, int steerCANCoderId) {
        steerPID = new RotationalPID(6/90., 0, 0, 6);
        
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
        DS_unoptimizedDesiredRotation = desiredState.angle.getDegrees();
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getRotation());
        updateDriveMotor(optimizedDesiredState.speedMetersPerSecond);
        
        if (desiredState.speedMetersPerSecond != 0) {
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
    
    public void updateDriveMotor (double desiredSpeedMetersPerSec) {
        DS_desiredDriveSpeed = desiredSpeedMetersPerSec;

        double voltsOutput = DRIVE_FEEDFORWARD.calculate(METERS_PER_SEC_TO_DRIVE_VOLTS * desiredSpeedMetersPerSec);
        DS_driveOutputVoltage = voltsOutput;

        driveMotor.setVoltage(DS_driveEnabled ? voltsOutput : 0);
    }
    
    public void updateSteerMotor (double voltage) {
        DS_steerOutputVoltage = voltage;
        steerMotor.setVoltage(DS_driveEnabled ? voltage : 0);
    }
    
    public void updateSteerMotor (Rotation2d desiredRotation) {
        DS_desiredRotation = desiredRotation.getDegrees();
        double voltsOutput = STEER_FEEDFORWARD.calculate(steerPID.calculate(getRotation(), desiredRotation));
        updateSteerMotor(voltsOutput);
    }
    
    private double getDisplacementMeters () {
        return driveMotor.getEncoder().getPosition() / 20.032;
    }
    
    public SwerveModulePosition getPosition () {
        return new SwerveModulePosition(
            getDisplacementMeters(),
            getRotation()
        );
    }
    
    /**
     * Stop all motor controllers assigned to this {@link SwerveModule}.
     */
    public void stop () {
        driveMotor.stopMotor();
        DS_desiredDriveSpeed = 0;
        DS_driveOutputVoltage = 0;
        steerMotor.stopMotor();
        DS_steerOutputVoltage = 0;
    }
    
    /**
     * Zero the steer encoder and save its offset to the encoder.
     */
    public void zeroSteerEncoder () {
        steerEncoder.zeroRotation();
    }
    
    private Rotation2d getRotation () {
        return steerEncoder.getRotation();
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("unop-desiredRotation", () -> DS_unoptimizedDesiredRotation, null);
        builder.addDoubleProperty("desiredRotation", () -> DS_desiredRotation, null);
        builder.addDoubleProperty("desiredDriveSpeed", () -> DS_desiredDriveSpeed, null);
        builder.addDoubleProperty("currentRotation", () -> steerEncoder.getRotation().getDegrees(), null);
        builder.addDoubleProperty("outputVoltage", () -> DS_steerOutputVoltage, null);

        builder.addBooleanProperty("Enabled Drive", () -> DS_driveEnabled, e -> DS_driveEnabled = e);
    }
    
}
