// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.EstimatedRobotPose;

/** Add your docs here. */
public class VisionMeasurement {
  public final EstimatedRobotPose pose;
  public final double avgTagDistance;
  public final int tagCount;

  public VisionMeasurement(EstimatedRobotPose pose, double avgTagDistance, int tagCount) {
    this.pose = pose;
    this.avgTagDistance = avgTagDistance;
    this.tagCount = tagCount;
  }
}
