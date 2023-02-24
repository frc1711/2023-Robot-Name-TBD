// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.hardware.Device;
import claw.hardware.Device.DeviceInitializer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    
    private static final double
        ENGAGE_VOLTAGE = 1,
        DISENGAGE_VOLTAGE = -4;
    
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
    
    private final Device<DigitalInput>
        lowerLimitSwitch = new Device<>(
            "DIO.LIMIT_SWITCH.INTAKE.LOWER_LIMIT",
            DigitalInput::new,
            DigitalInput::close
        ),
        upperLimitSwitch = new Device<>(
            "DIO.LIMIT_SWITCH.INTAKE.UPPER_LIMIT",
            DigitalInput::new,
            DigitalInput::close
        );
    
    private boolean isFullyEngaged = false;
    
    public Intake () {
        RobotContainer.putConfigSendable("IntakeSubsystem", this);
    }
    
    public enum IntakeEngagement {
        ENGAGE,
        DISENGAGE;
    }
    
    public void setIntakeEngagement (IntakeEngagement engagement) {
        if (engagement == IntakeEngagement.DISENGAGE) {
            isFullyEngaged = false;
        } else if (lowerLimitSwitch.get().get()) {
            isFullyEngaged = true;
        }
        
        double engageSpeed = 0;
        
        if (engagement == IntakeEngagement.ENGAGE) {
            if (!lowerLimitSwitch.get().get()) {
                engageSpeed = ENGAGE_VOLTAGE;
            }
        } else {
            if (!upperLimitSwitch.get().get()) {
                engageSpeed = DISENGAGE_VOLTAGE;
            }
        }
        
        leftEngage.get().setVoltage(engageSpeed);
        rightEngage.get().setVoltage(engageSpeed);
    }
    
    public void initSendable (SendableBuilder builder) {
        builder.addBooleanProperty("lower-limit", () -> lowerLimitSwitch.get().get(), null);
        builder.addBooleanProperty("upper-limit", () -> upperLimitSwitch.get().get(), null);
    }
    
    public void stopLowerBar () {
        setLowerBarSpeed(0);
    }
    
    public void stopTopBar () {
        setTopBarSpeed(0);
    }
    
    public void setTopBarSpeed (double input) {
        topRoller.get().setVoltage(input);
    }
    
    public void setLowerBarSpeed (double input) {
        bottomRoller.get().setVoltage(input);
    }
    
    public void stop () {
        stopLowerBar();
        stopTopBar();
    }
    
}