// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeEngagement;

public class RunTimedIntake extends CommandBase {

  private final Intake intake;
  private double durationInSeconds;
  private Timer timer;

  public RunTimedIntake(
    Intake intake,
     double durationInSeconds
  ) {
    this.intake = intake;
    this.durationInSeconds = durationInSeconds;  
    this.timer = new Timer();
    addRequirements(intake);  
  }


  
  @Override
  public void initialize() {
    intake.stop();
  }

  
  @Override
  public void execute() {
    intake.setIntakeEngagement(IntakeEngagement.ENGAGE);
    if (timer.hasElapsed(durationInSeconds)) intake.setIntakeEngagement(IntakeEngagement.DISENGAGE);
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
