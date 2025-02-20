package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTagOffsetManager {
    private static class Poses {
      private static final Pose2d DEFAULT_POSE = new Pose2d(0, -0.10, new Rotation2d());

      private static final Pose2d REEF_RIGHT = new Pose2d(0.1611, -0.10, new Rotation2d());
      private static final Pose2d REEF_LEFT = new Pose2d(-0.1611, -0.10, new Rotation2d());

      private static final Pose2d CLIMB_RIGHT = new Pose2d(1, -0.10, new Rotation2d()); // TODO: CHANGE WITH REAL VALUES
      private static final Pose2d CLIMB_MID = new Pose2d(-0.1611, -0.10, new Rotation2d());  // TODO: CHANGE WITH REAL VALUES
      private static final Pose2d CLIMB_LEFT = new Pose2d(-0.1611, -0.10, new Rotation2d());  // TODO: CHANGE WITH REAL VALUES
    }
    
    private enum PoseType {
      DEFAULT(new Pose2d[]{
        Poses.DEFAULT_POSE
      }), 

      REEF(new Pose2d[]{
        Poses.REEF_LEFT,
        Poses.REEF_RIGHT
      }), 

      PROCESSOR(new Pose2d[]{
        Poses.DEFAULT_POSE
      }),

      CLIMB(new Pose2d[]{
        Poses.CLIMB_RIGHT,
        Poses.CLIMB_MID,
        Poses.CLIMB_LEFT
      });

      private final Pose2d[] variants;
      PoseType(Pose2d[] variants) {
        this.variants = variants;
      }

      public boolean isVariantOutOfBorder(int variant) {
        return variant > variants.length-1 || variant < 0;
      }

      public Pose2d getVariant(int variant) {
        if (isVariantOutOfBorder(variant)) {
          variant = 0;
        }

        return variants[variant];
      }
    }

    private static final PoseType[] ID_TYPES = initializePoseTypes();
    
    private static PoseType[] initializePoseTypes() {
        PoseType[] poseTypes = new PoseType[23];

        for (int i = 6; i <= 11; i++) poseTypes[i] = PoseType.REEF;
        for (int i = 17; i <= 22; i++) poseTypes[i] = PoseType.REEF;

        poseTypes[1] = poseTypes[2] = poseTypes[11] = poseTypes[12] = PoseType.DEFAULT;
        poseTypes[3] = poseTypes[16] = PoseType.PROCESSOR;
        poseTypes[4] = poseTypes[5] = poseTypes[14] = poseTypes[15] = PoseType.CLIMB;

        return poseTypes;
    }
    
    public static Pose2d getPose(int id, int variant) {
      if (id < 1 || id > 22) return Poses.DEFAULT_POSE;
      
      PoseType type = ID_TYPES[id];
      return type.getVariant(variant);
  }
}
