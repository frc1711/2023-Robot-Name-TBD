// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeEngagement;
import frc.robot.subsystems.Intake.IntakeSpeedMode;

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
    timer.start();
    timer.reset();
  }

  
  @Override
  public void execute() {
    intake.setIntakeEngagement(IntakeEngagement.ENGAGE);
    intake.setIntakeSpeedMode(IntakeSpeedMode.CUBE);
    if (timer.hasElapsed(durationInSeconds))  {
      intake.setIntakeEngagement(IntakeEngagement.DISENGAGE);
      intake.setIntakeSpeedMode(IntakeSpeedMode.STOP);
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
