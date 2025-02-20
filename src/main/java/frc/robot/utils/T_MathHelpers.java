package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class T_MathHelpers {
  public static double clampSymmetric(double x, double range) {
    return MathUtil.clamp(x, -range, range);
  }

  public static double getSpeed(double value, double tolerance, double maxSpeed) {
    if (Math.abs(maxSpeed) > tolerance) {
      return clampSymmetric(value, maxSpeed);
    }

    return 0;
  }
}
