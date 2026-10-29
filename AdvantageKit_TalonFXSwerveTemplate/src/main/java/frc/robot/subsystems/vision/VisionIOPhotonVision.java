package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Real hardware IO -- a single PhotonVision coprocessor camera doing multi-tag PnP. */
public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    camera = new PhotonCamera(cameraName);

    AprilTagFieldLayout fieldLayout = null;
    try {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    } catch (Exception e) {
      System.out.println("Failed to load AprilTagFieldLayout: " + e.getMessage());
    }

    estimator = fieldLayout == null ? null : new PhotonPoseEstimator(fieldLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.hasEstimate = false;
    inputs.ambiguity = -1.0;

    if (estimator == null) {
      return;
    }

    // Drain every frame that arrived since last loop; keep the newest usable estimate.
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) {
        continue;
      }
      Optional<EstimatedRobotPose> estimatedPose = estimator.estimateCoprocMultiTagPose(result);
      if (estimatedPose.isEmpty()) {
        continue;
      }

      EstimatedRobotPose camPose = estimatedPose.get();
      inputs.hasEstimate = true;
      inputs.estimatedPose = camPose.estimatedPose.toPose2d();
      inputs.timestampSeconds = camPose.timestampSeconds;
      inputs.tagCount = camPose.targetsUsed.size();

      double avgDist = 0;
      for (PhotonTrackedTarget target : camPose.targetsUsed) {
        avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
      }
      inputs.avgTagDistanceMeters =
          camPose.targetsUsed.isEmpty() ? 0 : avgDist / camPose.targetsUsed.size();
      inputs.ambiguity = camPose.targetsUsed.get(0).getPoseAmbiguity();
    }
  }
}
