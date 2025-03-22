package frc.robot;

public enum Presets {
  NEUTRAL(0.00, 0.05),
  INTAKE(0.00, 0.05),

  L4CORAL(1.3, -3.35),
  L3CORAL(1.035, 0.05),
  L2CORAL(0.645, 0.05),
  L1CORAL(0.315, 0.3),

  BARGEALGAE(1.4, 2.5),
  L3ALGAE(0.85, 0.75),
  L2ALGAE(0.45, 0.75),
  PROCESSORALGAE(0.15, 0.0);

  public double extensionMeters;
  public double angleRads;

  Presets(double extensionMeters, double angleRads) {
    this.extensionMeters = extensionMeters;
    this.angleRads = angleRads;
  }
}
