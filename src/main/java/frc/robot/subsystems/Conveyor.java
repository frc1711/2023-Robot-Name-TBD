// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;

public class Conveyor extends SubsystemBase {
  
  private static Conveyor conveyorInstance;

  public static Conveyor getInstance () {
    if (conveyorInstance == null) conveyorInstance = new Conveyor(new CANSparkMax(IDMap.CONVEYOR, MotorType.kBrushless), 1);
    return conveyorInstance;
  }

  CANSparkMax conveyor;
  double multiplier; //TODO: Calculate this value

  public Conveyor(CANSparkMax conveyor,
                  double multiplier) {
    this.conveyor = conveyor;
  }

  public void set (double input) {
    conveyor.setVoltage(input * multiplier);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
