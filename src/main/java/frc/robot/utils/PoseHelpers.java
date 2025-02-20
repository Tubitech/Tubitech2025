package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseHelpers {
  public static Pose2d convertPose3dToPose2d(Pose3d pose3d) {
    double x = pose3d.getX();
    double y = pose3d.getY();
    
    Rotation2d rotation = new Rotation2d(pose3d.getRotation().getZ());

    return new Pose2d(x, y, rotation);
  }

  public static Pose2d combinePoses(Pose2d currentTarget, Pose2d offset) {
    Translation2d offsetTranslation = offset.getTranslation();
    Rotation2d offsetRotation = offset.getRotation();

    Translation2d newTranslation = currentTarget.getTranslation().plus(offsetTranslation);

    Rotation2d newRotation = currentTarget.getRotation().plus(offsetRotation);

    return new Pose2d(newTranslation, newRotation);
  }

}
