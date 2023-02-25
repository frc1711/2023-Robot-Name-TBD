// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.hardware.Device;
import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;

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
        CLAW_HOMING_VOLTAGE = 2.5;
    
    private static Arm armInstance;
    
    public static Arm getInstance () {
        if (armInstance == null) {
            armInstance = new Arm(
                new CANSparkMax(15, MotorType.kBrushless), 
                new CANSparkMax(14, MotorType.kBrushless), 
                new DigitalInput(IDMap.ARM_LIMIT_SWITCH)
            );
        }
        return armInstance;
    }
    
    private final CANSparkMax armMotor, clawMotor;
    private final DigitalInput armLimitSwitch;
    private final RelativeEncoder clawEncoder;
    
    private final Device<DutyCycle> armEncoder = new Device<>(
        "DIO.ENCODER.ARM.ARM_ENCODER",
        id -> new DutyCycle(new DigitalInput(2)),
        DutyCycle::close
    );
    
    private final Debouncer clawGrabDebouncer = new Debouncer(.2, DebounceType.kRising);
    private double clawEncoderOffset = 0;
    
    public Arm(CANSparkMax armMotor, CANSparkMax clawMotor, DigitalInput armLimitSwitch) {
        this.armMotor = armMotor;
        this.clawMotor = clawMotor;
        this.armLimitSwitch = armLimitSwitch;
        clawEncoder = clawMotor.getEncoder();
        
        XboxController controller = new XboxController(0);
        Transform transform = InputTransform.getInputTransform(
            InputTransform.SQUARE_CURVE,
            0.2
        );
        
        LiveCommandTester<XboxController> tester = new LiveCommandTester<>(
            () -> controller,
            c -> {
                operateClaw(ClawMovement.NONE);
                if (c.getLeftBumper()) {
                    setArmVoltage(7 * transform.apply(c.getRightY()));
                } else stopArm();
            },
            this::stop,
            this
        );
        
        CLAWRobot.getExtensibleCommandInterpreter().addCommandProcessor(
            tester.toCommandProcessor("armtest")
        );
        
        RobotContainer.putConfigSendable("Arm Subsystem", this);
    }
    
    public void setArmVoltage(double input) {
        armMotor.setVoltage(input);
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
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("Claw output current", clawMotor::getOutputCurrent, null);
        builder.addDoubleProperty("Arm position", () -> armEncoder.get().getOutput(), null);
    }
    
}