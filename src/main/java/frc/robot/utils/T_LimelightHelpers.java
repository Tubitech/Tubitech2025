package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.limelight.LimelightHelpers;
import frc.robot.dtos.Camera;

public class T_LimelightHelpers { 
  public static boolean isAprilTagInView(Camera camera) {
    return LimelightHelpers.getTV(camera.name());  
  }

  public static int getAprilTagIdInView(Camera camera) {
    return (int) LimelightHelpers.getFiducialID(camera.name());  
  }

  public static Pose2d getAprilTagePose2dByCamera(Camera camera) {
    return PoseHelpers.convertPose3dToPose2d(
      LimelightHelpers.getTargetPose3d_RobotSpace(camera.name())
    );

    // return convertPose3dToPose2d(
    //   LimelightHelpers.getTargetPose3d_CameraSpace(camera.name())
    // ).transformBy(camera.camOffset());  
  }

}