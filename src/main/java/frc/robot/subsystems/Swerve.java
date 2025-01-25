package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public static boolean gyroCheck;
    public static boolean ampGyroCheck;
    public static int ampMultiplierCheck;
    private boolean doRejectUpdate;
    public SwerveDrivePoseEstimator m_poseEstimator;
    public Pose2d mt2Pose;
    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.canBusName);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);


        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };


        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("/SmartDashboard/Drivetrain/Robot Pose", Pose2d.struct).publish();
StructPublisher<Pose2d> desiredAlignmentPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("/SmartDashboard/Drivetrain/Desired Alignment Pose", Pose2d.struct).publish();
StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("/SmartDashboard/Drivetrain/Desired States", SwerveModuleState.struct).publish();
StructArrayPublisher<SwerveModuleState> actualStatesPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("/SmartDashboard/Drivetrain/Actual States", SwerveModuleState.struct).publish();


    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
                mSwerveMods[0].getState(),
                mSwerveMods[1].getState(),
                mSwerveMods[2].getState(),
                mSwerveMods[3].getState());
    }


public SwerveDrivePoseEstimator getPoseEstimator() {
    m_poseEstimator =
    new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          mSwerveMods[0].getPosition(),
          mSwerveMods[1].getPosition(),
          mSwerveMods[2].getPosition(),
          mSwerveMods[3].getPosition()
        },
                new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        return m_poseEstimator;
    }
            public void getLimelightPose() {
    LimelightHelpers.SetRobotOrientation("limelight-front", getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
      if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        mt2Pose = mt2.pose;
      }
    }
    public void driveRobotRelative(ChassisSpeeds robotRelativeChassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(robotRelativeChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public boolean getGyroCheck() {
        return gyroCheck;
    }

    public boolean getAmpGyroCheck() {
        return ampGyroCheck;
    }

    public int ampCheckMultiplier() {
        return ampMultiplierCheck;
    }

    @Override
    public void periodic() {
        robotPosePublisher.set(getPose());
        // desiredAlignmentPosePublisher.set();
        // desiredStatesPublisher.set(getModuleStates());
        actualStatesPublisher.set(getModuleStates());
        var alliance = DriverStation.getAlliance();
        if (getHeading().getDegrees() >= -90 && getHeading().getDegrees() <= 90) {
            gyroCheck = false;
            SmartDashboard.putBoolean("GyroYaw", true);
        } else {
            gyroCheck = true;
            SmartDashboard.putBoolean("GyroYaw", false);
        }
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                if (getHeading().getDegrees() >= -180 && getHeading().getDegrees() <= 0) {
                    ampGyroCheck = false;
                    ampMultiplierCheck = 1;
                } else {
                    ampGyroCheck = true;
                    ampMultiplierCheck = -1;
                }
            } else {
                if (getHeading().getDegrees() >= 0 && getHeading().getDegrees() <= 180) {
                    ampGyroCheck = false;
                } else {
                    ampGyroCheck = true;
                }
            }
        } else {
            
        }

        swerveOdometry.update(getGyroYaw(), getModulePositions());
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        }

}