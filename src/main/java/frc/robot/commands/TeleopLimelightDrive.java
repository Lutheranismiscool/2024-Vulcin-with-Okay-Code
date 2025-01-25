// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
  Translation2d finalTranslation;
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
    LimelightHelpers.setLEDMode_ForceOn("limelight-front");
    // translationCoordinates = swerve.getLimelightPose();
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
        Translation2d translationVector = new Translation2d(0, 0.5);
        Rotation2d rotationVector = new Rotation2d(0);
        Pose2d finalVector = new Pose2d(translationVector, rotationVector);
        double translationAllowedX = 1;
        double translationAllowedY = 0;
        double translationCoordinatesX;
        double translationCoordinatesY;

    limelight.limelightTagMode(true);
    swerve.getLimelightPose();
    if (align == 1 | swerve.mt2Pose !=null) {
    // translationCoordinates = swerve.mt2Pose.minus(finalVector);
    translationCoordinatesX = swerve.mt2Pose.getX()-(translationAllowedX);
    translationCoordinatesY = swerve.mt2Pose.getY()-(translationAllowedY);
    translationCoordinates = new Translation2d(translationCoordinatesX, translationCoordinatesY);
    finalTranslation = new Translation2d(translationCoordinates.getX(), translationVector.getY());
    } else if (align == 0 | swerve.mt2Pose !=null) {
    translationCoordinatesX = swerve.mt2Pose.getX()+(translationAllowedX);
    translationCoordinatesY = swerve.mt2Pose.getY()+(translationAllowedY);
    translationCoordinates = new Translation2d(translationCoordinatesX, translationCoordinatesY);
    finalTranslation = new Translation2d(translationCoordinates.getX(), translationVector.getY());

        double strafeProportional;
        boolean strafeDisqualifier;
        Translation2d translationDifference;
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
        swerve.drive(finalTranslation, swerve.mt2Pose.getRotation().getDegrees(), true, true);
      }

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
