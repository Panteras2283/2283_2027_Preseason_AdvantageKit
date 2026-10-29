package frc.robot.subsystems.vision;

/**
 * Stub -- always reports no target, so in sim/replay the {@code Vision/CameraN} topics exist and
 * the pipeline runs but never gets a fix. Vision is validated on the real robot; a full simulated
 * pipeline (PhotonVision's VisionSystemSim projecting tags against a ground-truth pose) is a
 * separate effort if it's ever wanted.
 */
public class VisionIOSim implements VisionIO {}