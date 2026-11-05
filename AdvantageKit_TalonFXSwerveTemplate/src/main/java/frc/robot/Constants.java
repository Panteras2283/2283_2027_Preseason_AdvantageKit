// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class AutopilotConstants{

    public static final PathConstraints kPathConstraints =
      new PathConstraints(2.5, 4, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final double FIELD_LENGTH_METERS = 16.540;
    public static final double FIELD_WIDTH_METERS = 8.070;

    private static final Pose2d[][] POSES = new Pose2d[3][3];

    static {
      Rotation2d one = Rotation2d.fromDegrees(0);
      Rotation2d two = Rotation2d.fromDegrees(-120);
      Rotation2d three = Rotation2d.fromDegrees(-60);

      POSES[0][0] = new Pose2d(7, 6.767, Rotation2d.fromDegrees(0));
      POSES[0][1] = new Pose2d(4.0, 3.5, one);
      POSES[0][2] = new Pose2d(7, 1.127, Rotation2d.fromDegrees(45));

      POSES[1][0] = new Pose2d(4.6, 7.4, one);
      POSES[1][1] = new Pose2d(1.43, 7.31, Rotation2d.fromDegrees(-40));
      POSES[1][2] = new Pose2d(6.0, 3.0, two);

      POSES[2][0] = new Pose2d(5.0, 2.0, three);
      POSES[2][1] = new Pose2d(5.5, 2.5, three);
      POSES[2][2] = new Pose2d(6.0, 3.0, three);
    }

    public static Pose2d getPose(int side, int slot) {
      if (side < 0 || side >= POSES.length){
        System.out.println("[Autopilot] Invalid slot: " + slot);
        return new Pose2d();
      }
      if (slot < 0 || slot >= POSES[0].length) {
        System.out.println("[Autopilot] Invalid slot: " + slot);
        return new Pose2d();
      }

      Pose2d bluePose = POSES[side][slot];
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        return flipPoseToRed(bluePose);
      }
      return bluePose;
    }

    private static Pose2d flipPoseToRed(Pose2d bluePose){
      return new Pose2d(
        FIELD_LENGTH_METERS - bluePose.getX(), bluePose.getY(), new Rotation2d(Math.PI).minus(bluePose.getRotation()));
    }

    private AutopilotConstants(){}
  }

  private Constants(){}
}
