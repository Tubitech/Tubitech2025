// Author: @suayptalha

package frc.robot.commands.swervedrive.auto;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.utils.AprilTagOffsetManager;
import frc.robot.dtos.Camera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;


import static frc.robot.Constants.APRILTAG_DISTANCE_TOLERANCE;
import static frc.robot.Constants.APRILTAG_ANGLE_TOLERANCE;
import static frc.robot.Constants.MAX_SPEED;
import static frc.robot.Constants.ROTATION_SPEED;
import static frc.robot.Constants.CAMERAS;

import frc.robot.utils.PoseHelpers;
import frc.robot.utils.T_LimelightHelpers;
import frc.robot.utils.T_MathHelpers;

public class AlignToAprilTagCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final int aprilTagId;
  private final Pose2d positionOffset;

  public AlignToAprilTagCommand(SwerveSubsystem swerveSubsystem, int aprilTagId, int alignVariant) {
    this.swerveSubsystem = swerveSubsystem;
    this.aprilTagId = aprilTagId;
    this.positionOffset = AprilTagOffsetManager.getPose(aprilTagId, alignVariant);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    Pose2d targetPose = getTargetPose();
    driveToPose(targetPose);
  }

  @Override
  public boolean isFinished() {
    Pose2d tagPose = getTargetPose();
    return driveToPose(tagPose);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
  }
  
  public boolean driveToPose(Pose2d targetPose) {
    double xDistance = targetPose.getX();
    double yDistance = targetPose.getY();

    double xSpeed = getSpeedByDistance(xDistance);
    double ySpeed = getSpeedByDistance(yDistance);

    double rotationSpeed = getRotationSpeedByRadian(targetPose.getRotation().getRadians());
    
    if (xSpeed == 0 && ySpeed == 0 && rotationSpeed == 0) {
      return true;
    }

    swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);
    return false;
  }

  public boolean isAprilTagInViewTrue(Camera camera) {
    return T_LimelightHelpers.getAprilTagIdInView(camera) == this.aprilTagId;
  }

  public Pose2d getAprilTagePoseByCameras() {
    for (Camera camera : CAMERAS) {
        if (T_LimelightHelpers.isAprilTagInView(camera) &&
             isAprilTagInViewTrue(camera)) {
          return T_LimelightHelpers.getAprilTagePose2dByCamera(camera);
        }
    }

    return null;
  }

  public Pose2d getAprilTagPose() {
    Pose2d aprilTagPose2d;

    aprilTagPose2d = getAprilTagePoseByCameras();
    if (aprilTagPose2d != null) {
      return aprilTagPose2d;
    }

    return Vision.getAprilTagPose(aprilTagId, new Transform2d())
    .relativeTo(swerveSubsystem.getPose());
  }

  public Pose2d getTargetPose() {
    Pose2d aprilPose2d = getAprilTagPose();
    return PoseHelpers.combinePoses(aprilPose2d, positionOffset);
  }
  
  public static double getSpeedByDistance(double distance) {
    return T_MathHelpers.getSpeed(distance, APRILTAG_DISTANCE_TOLERANCE, MAX_SPEED);
  }

  public static double getRotationSpeedByRadian(double radian) {
    return T_MathHelpers.getSpeed(radian, APRILTAG_ANGLE_TOLERANCE, ROTATION_SPEED);
  }
}
 