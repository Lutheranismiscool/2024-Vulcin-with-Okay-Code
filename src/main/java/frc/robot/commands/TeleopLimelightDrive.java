// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TeleopLimelightDrive extends Command {
  Swerve swerve;
  Limelight limelight;
  IntakePivot intake;
  Arm arm;
  boolean amp;
  ChassisSpeeds relativeSpeed;
  boolean gyro;
  int invert;
  int align;
  Translation2d translation;
  Translation2d translationCoordinates;
  /** Creates a new TeleopLimelightDrive. */
  public TeleopLimelightDrive(Swerve swerve, Limelight limelight, int align) {
    this.swerve = swerve;
    this.align = align;
    this.limelight = limelight;
    addRequirements(swerve, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(swerve.getHeading().getDegrees() >= -90 && swerve.getHeading().getDegrees() <= 90){
    //   gyro = false;
    //   invert = -1;
    // } else {
    //   gyro = true;
    //   invert = 1;
    // }
    limelight.limelightTagMode(true);
    translationCoordinates = new Translation2d();
    swerve.getLimelightPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // double strafeVal;
        // if(swerve.getHeading().getDegrees() >= -90 && swerve.getHeading().getDegrees() <= 90){
        //   invert = 1;
        // } else {
        //   invert = -1;
        // }
            /* Get Values, Deadband*/
        // double translationVal = limelight.limelight_range_proportional() * invert;
        double strafeProportional;
        boolean strafeDisqualifier;
        if (align == 1) {
          // strafeVal = limelight.limelight_strafe_proportional() - 1;
          strafeProportional = limelight.limelight_strafe_proportional();
          strafeDisqualifier = true;
          SmartDashboard.putNumber("strafeProportional", strafeProportional);
          SmartDashboard.putBoolean("strafeDisqualifier", true);
          SmartDashboard.putNumber("align", align);
        } else {
          // strafeVal = limelight.limelight_strafe_proportional() + 1;
          strafeProportional = limelight.limelight_strafe_proportional();
          strafeDisqualifier = false;
          SmartDashboard.putNumber("strafeProportional", strafeProportional);
          SmartDashboard.putBoolean("strafeDisqualifier", false);
          SmartDashboard.putNumber("align", align);
        }
          
        // double strafeVal = limelight.limelight_strafe_proportional();
        // double rotationVal = limelight.limelight_aim_proportional();
        
        // relativeSpeed = new ChassisSpeeds(0, strafeVal, 0);
        /* Drive */
        swerve.drive(translationCoordinates, swerve.getLimelightPose().getRotation().getDegrees(), true, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.limelightTagMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
