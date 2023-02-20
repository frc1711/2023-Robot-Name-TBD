// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;

public class Arm extends SubsystemBase {
    
    private static final double HOMING_OUTPUT_CURRENT = 5, CLAW_HOMING_SPEED = 0.1;
    
    private static Arm armInstance;
    
    public static Arm getInstance () {
        if (armInstance == null) {
            armInstance = new Arm(
                new CANSparkMax(IDMap.ARM, MotorType.kBrushless), 
                new CANSparkMax(IDMap.CLAW, MotorType.kBrushless), 
                new DigitalInput(IDMap.ARM_LIMIT_SWITCH)
            );
        }
        return armInstance;
    }
    
    private final CANSparkMax armMotor, clawMotor;
    private final DigitalInput armLimitSwitch;
    private final RelativeEncoder clawEncoder;
    
    private final Debouncer clawGrabDebouncer = new Debouncer(.2, DebounceType.kRising);
    
    public Arm(CANSparkMax armMotor, CANSparkMax clawMotor, DigitalInput armLimitSwitch) {
        this.armMotor = armMotor;
        this.clawMotor = clawMotor;
        this.armLimitSwitch = armLimitSwitch;
        clawEncoder = clawMotor.getEncoder();
    }
    
    public void setArmSpeed(double input) {
        if (armLimitSwitch.get())
            stopArm();
        else
            arm.setVoltage(input * multiplier);
    }
    
    public void stopArm() {
        arm.setVoltage(0);
    }
    
    public boolean runClawHomingSequence () {
        if (claw.getOutputCurrent() > HOMING_OUTPUT_CURRENT) {
            stopClaw();
            
            return true;
        } else {
            claw.setVoltage(CLAW_HOMING_SPEED);
            return false;
        }
    }
    
    public void stopClaw() {
        claw.setVoltage(0);
    }
    
    public void stop() {
        stopArm();
        stopClaw();
    }
    
    @Override
    public void periodic() {
        
    }
}