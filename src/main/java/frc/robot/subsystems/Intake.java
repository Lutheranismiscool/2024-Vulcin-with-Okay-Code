// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkFlex reverseIntakeMotor = new SparkFlex(2, MotorType.kBrushless);
    private SparkFlex intakeMotor = new SparkFlex(3, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
  /** Creates a new Intake. */
  public Intake() {
    config.idleMode(IdleMode.kBrake);
    reverseIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void intake(double speed, boolean motor) {
    if(motor == false) {
    reverseIntakeMotor.set(-speed);
    } else if (motor == true) {
    intakeMotor.set(speed);
    }
  }
  public void manualIntake(double speed) {
    reverseIntakeMotor.set(-speed);
    intakeMotor.set(speed);
  }
  public void shoot(double speed, boolean motor) {
    if(motor == true) {
      reverseIntakeMotor.set(speed);
      } else if (motor == false) {
      intakeMotor.set(-speed);
      }
  }
}
