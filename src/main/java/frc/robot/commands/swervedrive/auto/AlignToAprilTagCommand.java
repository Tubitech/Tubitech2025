// Author: @suayptalha

package frc.robot.commands.swervedrive.auto;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform2d;

public class AlignToAprilTagCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Transform2d robotOffset;

  private final int aprilTagId;

  public AlignToAprilTagCommand(SwerveSubsystem swerveSubsystem, int aprilTagId) {
    this.swerveSubsystem = swerveSubsystem;
    this.robotOffset = new Transform2d(new Translation2d(0, 0), new Rotation2d(0)); //Burayı değiştirebilirsiniz. Şimdilik kamera offset'ini (0,0) verdim.
    this.aprilTagId = aprilTagId;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Pose2d tagPose = Vision.getAprilTagPose(aprilTagId, robotOffset);

    if (tagPose != null) {
      double xDistance = tagPose.getX();

      if (Math.abs(xDistance) < Constants.ALIGN_TOLERANCE) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
      } else {
        double translationVal = MathUtil.clamp(xDistance * Constants.ROTATION_SPEED, -Constants.ROTATION_SPEED, Constants.ROTATION_SPEED);
        Translation2d translation = new Translation2d(translationVal, 0);
        swerveSubsystem.drive(translation, 0, true);
      }
    } else {
      swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d tagPose = Vision.getAprilTagPose(aprilTagId, robotOffset);
    
    if (tagPose != null) {
      double xDistance = tagPose.getX();
      return Math.abs(xDistance) < Constants.ALIGN_TOLERANCE;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
  }
}
