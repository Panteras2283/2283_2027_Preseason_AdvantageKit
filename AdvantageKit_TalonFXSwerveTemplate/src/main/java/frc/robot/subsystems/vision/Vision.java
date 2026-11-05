package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Owns however many cameras are plugged in (Limelight, PhotonVision, or both -- see RobotContainer)
 * and feeds every accepted estimate into {@link Drive#addVisionMeasurement}.
 *
 * <p>Every loop, for each camera, logs the raw estimate ({@code Vision/CameraN/*} via {@code
 * processInputs}) plus whether it was accepted, why it was rejected if not, and the std-devs
 * applied. The {@code Vision/Summary/*} arrays are meant to be dragged straight onto the
 * AdvantageScope field view to see accepted vs. rejected fixes against {@code Odometry/Robot}.
 */
public class Vision extends SubsystemBase {
  private static final double MAX_ANGULAR_RATE_ROT_PER_SEC = 2.0;
  private static final double FIELD_MARGIN_METERS = 0.5;

  private enum RejectionReason {
    NONE,
    NO_ESTIMATE,
    HIGH_ANGULAR_RATE,
    OFF_FIELD
  }

  private final Drive drive;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  public Vision(Drive drive, VisionIO... io) {
    this.drive = drive;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    double headingDeg = drive.getPose().getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(drive.getChassisSpeeds().omegaRadiansPerSecond);

    List<Pose2d> allPoses = new ArrayList<>();
    List<Pose2d> acceptedPoses = new ArrayList<>();
    List<Pose2d> rejectedPoses = new ArrayList<>();

    for (int i = 0; i < io.length; i++) {
      io[i].setRobotHeading(headingDeg);
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);

      RejectionReason reason = classify(inputs[i], omegaRps);
      boolean accepted = reason == RejectionReason.NONE;
      Matrix<N3, N1> stdDevs = accepted ? getStdDevs(inputs[i]) : VecBuilder.fill(0.0, 0.0, 0.0);

      if (inputs[i].hasEstimate) {
        allPoses.add(inputs[i].estimatedPose);
        (accepted ? acceptedPoses : rejectedPoses).add(inputs[i].estimatedPose);
      }

      if (accepted) {
        drive.addVisionMeasurement(inputs[i].estimatedPose, inputs[i].timestampSeconds, stdDevs);
      }

      Logger.recordOutput("Vision/Camera" + i + "/Accepted", accepted);
      Logger.recordOutput("Vision/Camera" + i + "/RejectionReason", reason.toString());
      Logger.recordOutput(
          "Vision/Camera" + i + "/StdDevsApplied",
          new double[] {stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0)});
    }

    Logger.recordOutput("Vision/Summary/AllRobotPoses", allPoses.toArray(new Pose2d[0]));
    Logger.recordOutput("Vision/Summary/AcceptedRobotPoses", acceptedPoses.toArray(new Pose2d[0]));
    Logger.recordOutput("Vision/Summary/RejectedRobotPoses", rejectedPoses.toArray(new Pose2d[0]));
  }

  private RejectionReason classify(VisionIOInputsAutoLogged in, double omegaRps) {
    if (!in.hasEstimate) {
      return RejectionReason.NO_ESTIMATE;
    }
    // Camera blur / latency makes fixes unreliable while spinning fast.
    if (Math.abs(omegaRps) >= MAX_ANGULAR_RATE_ROT_PER_SEC) {
      return RejectionReason.HIGH_ANGULAR_RATE;
    }
    // A flipped-tag solution often lands the robot outside the field -- a cheap sanity gate.
    double x = in.estimatedPose.getX();
    double y = in.estimatedPose.getY();
    if (x < -FIELD_MARGIN_METERS
        || x > Constants.AutopilotConstants.FIELD_LENGTH_METERS + FIELD_MARGIN_METERS
        || y < -FIELD_MARGIN_METERS
        || y > Constants.AutopilotConstants.FIELD_WIDTH_METERS + FIELD_MARGIN_METERS) {
      return RejectionReason.OFF_FIELD;
    }
    return RejectionReason.NONE;
  }

  /**
   * Distance/tag-count based trust model. The original codebase had a separate, more elaborate
   * formula per provider (Limelight vs. PhotonVision); this is a single blended version since both
   * now flow through the same {@link VisionIO} shape -- revisit if one provider turns out to need
   * materially different trust tuning.
   */
  private Matrix<N3, N1> getStdDevs(VisionIOInputsAutoLogged in) {
    double xyStds;
    double degStds;

    if (in.tagCount >= 2) {
      xyStds = 0.05;
      degStds = 3;
    } else if (in.ambiguity > 0.2) {
      // Ambiguous single-tag solution (PhotonVision-reported) -- could be flipped, so
      // mathematically ignore the translation.
      xyStds = 100.0;
      degStds = 30;
    } else {
      double dist = in.avgTagDistanceMeters;
      xyStds = 0.5 + (0.1 * dist);
      degStds = 5 + (2 * dist);
    }

    return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
  }
}
