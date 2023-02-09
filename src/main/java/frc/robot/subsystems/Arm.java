// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;

public class Arm extends SubsystemBase {
  
  private static Arm armInstance;

  public static Arm getInstance () {
    if (armInstance == null) armInstance = new Arm(
                                    new CANSparkMax(IDMap.ARM, MotorType.kBrushless), 
                                    new CANSparkMax(IDMap.CLAW, MotorType.kBrushless), 
                                    new DigitalInput(IDMap.ARM_LIMIT_SWITCH)
                                    );
    return armInstance;
  }

  private CANSparkMax arm, claw;
  private DigitalInput limitSwitch;
  private double multiplier;

  public Arm(CANSparkMax arm, CANSparkMax claw, DigitalInput limitSwitch) {
    this.arm = arm;
    this.claw = claw;
  }

  public void setArmSpeed(double input) {
    if (limitSwitch.get()) {
      stop();
    }
    else arm.setVoltage(input * multiplier);
  }

  public void stop () {
    arm.setVoltage(0);
  }

  public void setClawSpeed (double input) {
  }
  @Override
  public void periodic() {

  }
}