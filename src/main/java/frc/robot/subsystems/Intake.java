// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.hardware.Device;
import claw.hardware.LimitSwitchDevice;
import claw.hardware.Device.DeviceInitializer;
import claw.hardware.LimitSwitchDevice.NormalState;
import claw.math.Transform;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    
    // Cubes: -2.3, -2.8
    // Cones: -3.8, -4.5
    private static final double
        INTAKE_TOP_FORWARD_VOLTAGE = -3.8,
        INTAKE_BOTTOM_FORWARD_VOLTAGE = -4.5;
    
    private static Intake intakeInstance;
    
    public static Intake getInstance() {
        if (intakeInstance == null)
            intakeInstance = new Intake();
        return intakeInstance;
    }
    
    private static DeviceInitializer<CANSparkMax> getSparkMaxInitializer (IdleMode idleMode) {
        return (id) -> {
            CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
            motor.clearFaults();
            motor.setIdleMode(idleMode);
            return motor;
        };
    }
    
    private static void finalizeSparkMax (CANSparkMax motor) {
        motor.stopMotor();
        motor.close();
    }
    
    private final Device<CANSparkMax>
        topRoller = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.TOP_ROLLER",
            getSparkMaxInitializer(IdleMode.kCoast),
            Intake::finalizeSparkMax
        ),
        bottomRoller = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.BOTTOM_ROLLER",
            getSparkMaxInitializer(IdleMode.kCoast),
            Intake::finalizeSparkMax
        ),
        leftEngage = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.LEFT_ENGAGE",
            getSparkMaxInitializer(IdleMode.kBrake),
            Intake::finalizeSparkMax
        ),
        rightEngage = new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE.RIGHT_ENGAGE",
            getSparkMaxInitializer(IdleMode.kBrake),
            Intake::finalizeSparkMax
        );
    
    private final LimitSwitchDevice
        lowerLimitSwitch = new LimitSwitchDevice("DIO.LIMIT_SWITCH.INTAKE.LOWER_LIMIT", NormalState.NORMALLY_CLOSED),
        upperLimitSwitch = new LimitSwitchDevice("DIO.LIMIT_SWITCH.INTAKE.UPPER_LIMIT", NormalState.NORMALLY_CLOSED);
    
    public Intake () {
        RobotContainer.putConfigSendable("Intake Subsystem", this);
    }
    
    public boolean isLowerPressed () {
        return !lowerLimitSwitch.isPressed();
    }
    
    public boolean isUpperPressed () {
        return !upperLimitSwitch.isPressed();
    }
    
    public enum IntakeMode {
        FORWARD (1),
        REVERSE (-1),
        STOP (0);
        
        private final double speedMult;
        private IntakeMode (double speedMult) {
            this.speedMult = speedMult;
        }
    }
    
    public void setIntakeMode (IntakeMode mode) {
        topRoller.get().setVoltage(mode.speedMult * INTAKE_TOP_FORWARD_VOLTAGE);
        bottomRoller.get().setVoltage(mode.speedMult * INTAKE_BOTTOM_FORWARD_VOLTAGE);
    }
    
    public double getEngagementVelocity () {
        return leftEngage.get().getEncoder().getVelocity() - rightEngage.get().getEncoder().getVelocity();
    }
    
    public void setEngagementVoltage (double voltage) {
        leftEngage.get().setVoltage(voltage);
        rightEngage.get().setVoltage(-voltage);
    }
    
    public void stopEngagementMotors () {
        leftEngage.get().stopMotor();
        rightEngage.get().stopMotor();
    }
    
    public void stop () {
        stopEngagementMotors();
        setIntakeMode(IntakeMode.STOP);
    }
    
    public void initSendable (SendableBuilder builder) {
        builder.addBooleanProperty("lower-limit", this::isLowerPressed, null);
        builder.addBooleanProperty("upper-limit", this::isUpperPressed, null);
    }
    
}