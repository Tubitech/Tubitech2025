package frc.robot.dtos;

import edu.wpi.first.math.geometry.Transform2d;

public record Camera (String name, Transform2d camOffset) {};
