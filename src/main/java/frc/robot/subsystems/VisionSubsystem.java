// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {  
  private final List<PhotonCamera> cameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();

  public VisionSubsystem() {
    AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    for (int i = 0; i < Constants.VisionConstants.cameraNames.length; i++) {

      PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.cameraNames[i]);

      PhotonPoseEstimator estimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.VisionConstants.cameraPositions[i]
      );

      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      cameras.add(camera);
      poseEstimators.add(estimator);
    }
  }

  public List<VisionMeasurement> getVisionMeasurements() {
    List<VisionMeasurement> measurements = new ArrayList<>();

    for (int i = 0; i < cameras.size(); i++) {
      PhotonCamera camera = cameras.get(i);
      PhotonPoseEstimator estimator = poseEstimators.get(i);
      // Drain each camera once per loop so frames are neither skipped nor reused.
      for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
        if (!result.hasTargets()
            || result.getTargets().size() < Constants.VisionConstants.MIN_TAGS) {
          continue;
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget.getPoseAmbiguity() > Constants.VisionConstants.MAX_AMBIGUITY) {
          continue;
        }

        double avgDistance = 0;
        for (PhotonTrackedTarget target : result.getTargets()) {
          avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDistance /= result.getTargets().size();
        if (avgDistance > Constants.VisionConstants.MAX_DISTANCE_METERS) {
          continue;
        }

        var estimatedPose = estimator.update(result);
        if (estimatedPose.isPresent()) {
          measurements.add(new VisionMeasurement(
              estimatedPose.get(), avgDistance, result.getTargets().size()));
        }
      }
    }

    // Camera queues can overlap in time; fuse their observations chronologically.
    measurements.sort(java.util.Comparator.comparingDouble(m -> m.pose.timestampSeconds));
    return measurements;
  }
}
