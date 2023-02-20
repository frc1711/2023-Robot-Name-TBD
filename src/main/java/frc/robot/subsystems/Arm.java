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
    
    /**
     * An offset in the claw's relative encoder's reading from the homing spot (fully collapsed) to the maximum
     * extension of the claw (fully released). This offset helps to prevent the claw from extending too far and
     * breaking itself.
     */
    private static final double CLAW_MAX_REACH_OFFSET = 30;
    
    /**
     * An offset in the claw's relative encoder's reading from the homing spot (fully collapsed) to the minimum
     * extension of the claw (almost fully grabbing). This offset helps to prevent the claw from contracting too
     * much and breaking itself.
     */
    private static final double CLAW_MIN_REACH_OFFSET = 2;
    
    // TODO: Add javadocs
    private static final double GRAB_OUTPUT_CURRENT = 5;
    
    private static final double
        CLAW_MOVE_VOLTAGE = 0.3,
        CLAW_HOMING_VOLTAGE = 0.1;
    
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
    private double clawEncoderOffset = 0;
    
    public Arm(CANSparkMax armMotor, CANSparkMax clawMotor, DigitalInput armLimitSwitch) {
        this.armMotor = armMotor;
        this.clawMotor = clawMotor;
        this.armLimitSwitch = armLimitSwitch;
        clawEncoder = clawMotor.getEncoder();
    }
    
    public void setArmSpeed(double input) {
        // TODO: Find a decent speed multiplier
        if (armLimitSwitch.get())
            stopArm();
        else
            armMotor.setVoltage(0);
    }
    
    public void stopArm() {
        armMotor.setVoltage(0);
    }
    
    public boolean runClawHomingSequence () {
        if (clawMotor.getOutputCurrent() > GRAB_OUTPUT_CURRENT) {
            clawEncoderOffset = clawEncoder.getPosition();
            clawMotor.setVoltage(0);
            return true;
        } else {
            clawMotor.setVoltage(CLAW_HOMING_VOLTAGE);
            return false;
        }
    }
    
    private boolean clawCanContract () {
        return clawEncoder.getPosition() - clawEncoderOffset > CLAW_MIN_REACH_OFFSET;
    }
    
    private boolean clawCanExtend () {
        return clawEncoder.getPosition() - clawEncoderOffset < CLAW_MAX_REACH_OFFSET;
    }
    
    public void operateClaw (ClawMovement move) {
        boolean isHoldingObject = clawGrabDebouncer.calculate(clawMotor.getOutputCurrent() > GRAB_OUTPUT_CURRENT);
        switch (move) {
            case NONE:
                clawMotor.stopMotor();
                break;
            case GRAB:
                if (isHoldingObject || !clawCanContract()) clawMotor.stopMotor();
                else clawMotor.setVoltage(CLAW_MOVE_VOLTAGE);
                break;
            case RELEASE:
                if (!clawCanExtend()) clawMotor.stopMotor();
                else clawMotor.setVoltage(-CLAW_MOVE_VOLTAGE);
                break;
        }
    }
    
    public enum ClawMovement {
        NONE,
        GRAB,
        RELEASE;
    }
    
    public void stop() {
        stopArm();
        operateClaw(ClawMovement.NONE);
    }
    
}