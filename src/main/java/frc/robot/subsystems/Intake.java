// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.hardware.Device;
import claw.hardware.LimitSwitchDevice;
import claw.hardware.LimitSwitchDevice.NormalState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private static Intake intakeInstance;
    
    public static Intake getInstance() {
        if (intakeInstance == null) {
            intakeInstance = new Intake(
                initializeMotor("LEFT_ARM_RELEASE", IdleMode.kBrake),
                initializeMotor("RIGHT_ARM_RELEASE", IdleMode.kBrake),
                initializeMotor("UPPER_INTAKE", IdleMode.kCoast),
                initializeMotor("LOWER_INTAKE", IdleMode.kCoast),
                initializeDIO("UPPER_INTAKE_LIMIT_SWITCH"),
                initializeDIO("LOWER_INTAKE_LIMIT_SWTICH")
            );
        }
        return intakeInstance;
    }

    private static LimitSwitchDevice initializeDIO (String partName) {
        return new LimitSwitchDevice("PWM.LIMIT_SWITCH.INTAKE."+partName, NormalState.NORMALLY_OPEN );
    }

    private static Device<CANSparkMax> initializeMotor (String partName, IdleMode idleMode) {
        return new Device<>(
            "CAN.MOTOR_CONTROLLER.INTAKE." + partName, 
            id -> {
                CANSparkMax motor = new CANSparkMax (id, MotorType.kBrushless);
                motor.setIdleMode(idleMode);
                return motor;
            }, 
            motor -> {
                motor.stopMotor();
                motor.close();
        });
    }
    
    private final Device<CANSparkMax> leftDeployMotor, rightDeployMotor, topBar, lowerBar;
    private final Device<DigitalInput> upperLimitSwitch, lowerLimitSwitch;
    
    public Intake(
            Device<CANSparkMax> leftDeployMotor,
            Device<CANSparkMax> rightDeployMotor,
            Device<CANSparkMax> topBar,
            Device<CANSparkMax> lowerBar,
            Device<DigitalInput> upperLimitSwitch,
            Device<DigitalInput> lowerLimitSwitch
            ) {
        this.leftDeployMotor = leftDeployMotor;
        this.rightDeployMotor = rightDeployMotor;
        this.topBar = topBar;
        this.lowerBar = lowerBar;
        this.upperLimitSwitch = upperLimitSwitch;
        this.lowerLimitSwitch = lowerLimitSwitch;
    }

    /**
     * Use this method to deploy or retract the intake. 
     */
    public void operateIntake () {
        double intakeDirection;
        if (upperLimitSwitch.get().get()) intakeDirection = 1;
        else if (lowerLimitSwitch.get().get()) intakeDirection = -1;
        else intakeDirection = 0;
        leftDeployMotor.get().setVoltage((12*.2) * intakeDirection);
        rightDeployMotor.get().setVoltage((12*.2) * intakeDirection);
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
    
    public void stop () {
        stopLowerBar();
        stopTopBar();
    }
    
}