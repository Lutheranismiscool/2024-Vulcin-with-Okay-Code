// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TeleopShooter extends Command {
  private double shooterSpeedSup;
  private Intake shooter;
  private int counter = 0;
  private int target = 0;
  BooleanSupplier motorSide;
  /** Creates a new Shooter. */
  public TeleopShooter(Intake shooter, double shooterSpeedSup, double seconds, BooleanSupplier motorSide) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shooterSpeedSup = shooterSpeedSup;
    this.motorSide = motorSide;
    target = (int)( seconds * 50 );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterSpeedVal = shooterSpeedSup;
    if(counter < target) {
      counter++;
      shooter.shoot(shooterSpeedVal, motorSide.getAsBoolean());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0, motorSide.getAsBoolean());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= target;
  }
}
