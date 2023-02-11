// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;
import frc.robot.util.NumericDebouncer;

public class Arm extends SubsystemBase {
  
  private static Arm armInstance;

  public static Arm getInstance () {
    if (armInstance == null) armInstance = new Arm(
                                    new CANSparkMax(IDMap.ARM, MotorType.kBrushless), 
                                    new CANSparkMax(IDMap.CLAW, MotorType.kBrushless), 
                                    new DigitalInput(IDMap.ARM_LIMIT_SWITCH),
                                    new DigitalInput(IDMap.CLAW_LIMIT_SWITCH)
                                    );
    return armInstance;
  }

  private CANSparkMax arm, claw;
  private DigitalInput armLimitSwitch, clawLimitSwitch;
  private double multiplier = 12;
  private NumericDebouncer clawDebouncer = new NumericDebouncer(new Debouncer(multiplier, DebounceType.kRising));

  public Arm(CANSparkMax arm, CANSparkMax claw, DigitalInput armLimitSwitch, DigitalInput clawLimitSwitch) {
    this.arm = arm;
    this.claw = claw;
    this.clawLimitSwitch = clawLimitSwitch;
    this.armLimitSwitch = armLimitSwitch;
  }

  public void setArmSpeed(double input) {
    if (armLimitSwitch.get()) {
      stop();
    }
    else arm.setVoltage(input * multiplier);
  }

  public void stop () {
    arm.setVoltage(0);
  }

  public void operateClaw (double input) {
    double outputCurrent;
    if (claw.getOutputCurrent() != 0) {
      outputCurrent = clawDebouncer.calculate(Optional.of(claw.getOutputCurrent())).get();
    }
    else outputCurrent = clawDebouncer.calculate(Optional.empty()).get();
    if (outputCurrent >= 8) stopClaw();
    else claw.setVoltage(input);
  }

  public void stopClaw() {
    claw.setVoltage(0);
  }

  public void stopAll() {
    stop();
    stopClaw();
  }

  @Override
  public void periodic() {

  }
}