// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.hardware.Device;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private static Intake intakeInstance;
    
    public static Intake getInstance() {
        if (intakeInstance == null) {
            intakeInstance = new Intake(
                initializeMotor("LEFT_ARM_RELEASE"),
                initializeMotor("RIGHT_ARM_RELEASE"),
                initializeMotor("UPPER_INTAKE"),
                initializeMotor("LOWER_INTAKE"),
                initializeDIO("UPPER_INTAKE_LIMIT_SWITCH"),
                initializeDIO("LOWER_INTAKE_LIMIT_SWTICH")
            );
        }
        return intakeInstance;
    }

    private static Device<DigitalInput> initializeDIO (String partName) {
        return new Device<> (
            "PWM.LIMIT_SWITCH.INTAKE." + partName,
            id -> new DigitalInput(id),
            null
        );
    }

    private static Device<CANSparkMax> initializeMotor (String partName) {
        return new Device<>(
            "CAN.MOTOR.CONTROLLER.INTAKE." + partName, 
            id -> {
                CANSparkMax motor = new CANSparkMax (id, MotorType.kBrushless);
                motor.setIdleMode(IdleMode.kBrake);
                return motor;
            }, 
            motor -> {
                motor.stopMotor();
                motor.close();
        });
    }
    
    private final Device<CANSparkMax> leftArm, rightArm, topBar, lowerBar;
    private final Device<DigitalInput> leftLimitSwitch, rightLimitSwitch;
    
    public Intake(
            Device<CANSparkMax> leftArm,
            Device<CANSparkMax> rightArm,
            Device<CANSparkMax> topBar,
            Device<CANSparkMax> lowerBar,
            Device<DigitalInput> leftLimitSwitch,
            Device<DigitalInput> rightLimitSwitch
            ) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        this.topBar = topBar;
        this.lowerBar = lowerBar;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
    }
    
    public void raiseArmUnbound (double input) {
        leftArm.get().setVoltage(input);
        rightArm.get().setVoltage(input);
    }
    
    public void operateArmBound (double input) {
        if (leftLimitSwitch.get().get() || rightLimitSwitch.get().get()) {
            stopArm();
        } else {
            leftArm.get().setVoltage(input);
            rightArm.get().setVoltage(input);
        }
    }
    
    public void stopArm() {
        leftArm.get().setVoltage(0);
        rightArm.get().setVoltage(0);
    }
    
    public void stopTopBar () {
        topBar.get().setVoltage(0);
    }
    
    public void stopLowerBar () {
        lowerBar.get().setVoltage(0);
    }
    
    public void setTopBarSpeed (double input) {
        topBar.get().setVoltage(input);
    }
    
    public void setLowerBarSpeed (double input) {
        lowerBar.get().setVoltage(input);
    }
    
    public void stopAll () {
        stopArm();
        stopLowerBar();
        stopTopBar();
    }
    
}