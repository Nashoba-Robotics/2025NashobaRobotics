package frc.robot;

public enum Presets {
  NEUTRAL(-0.0005, 0.02),
  INTAKE(-0.0005, 0.02),

  L4CORAL(1.3, -3.45),
  L3CORAL(1.035, 0.05),
  L2CORAL(0.645, 0.05),
  L1CORAL(0.315, 0.38),

  BARGEALGAE(1.4, 2.5),
  L3ALGAE(0.85, 0.75),
  L2ALGAE(0.45, 0.75),
  PROCESSORALGAE(0.18, 0.10);

  public double extensionMeters;
  public double angleRads;

  Presets(double extensionMeters, double angleRads) {
    this.extensionMeters = extensionMeters;
    this.angleRads = angleRads;
  }
}
