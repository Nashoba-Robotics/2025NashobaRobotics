package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

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

  public static final class Elevator {
    public static final String CANBUS = "";
    public static final int LEADER_ID = 0;
    public static final int FOLLOWER_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 0;
    public static final double SUPPLY_LIMIT = 0;

    public static final double GEAR_RATIO = 0;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
    public static final double MOTION_MAGIC_ACCELERATION = 0.0;

    public static final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromRadians(0.0);
    public static final Rotation2d REVERSE_SOFT_LIMIT = Rotation2d.fromRadians(0.0);

    public static final Slot0Configs PID =
        new Slot0Configs()
            .withKS(0.0)
            .withKG(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static);
  }

  public static final class Manipulator {
    public static final String CANBUS = "";
    public static final int MANIPULATOR_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 0;
    public static final double SUPPLY_LIMIT = 0;

    public static final double GEAR_RATIO = 0;

    public static final double MOTION_MAGIC_ACCELERATION = 0.0;

    public static final Slot0Configs PID =
        new Slot0Configs().withKS(0.0).withKV(0.0).withKA(0.0).withKP(0.0).withKI(0.0).withKD(0.0);
  }

  public static final class Climber {
    public static final String CANBUS = "";
    public static final int PIVOT_ID = 0;
    public static final int GRABBER_ID = 0;

    public static final InvertedValue PIVOT_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double PIVOT_STATOR_LIMIT = 0;
    public static final double PIVOT_SUPPLY_LIMIT = 0;

    public static final double PIVOT_GEAR_RATIO = 0;

    public static final double PIVOT_MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
    public static final double PIVOT_MOTION_MAGIC_ACCELERATION = 0.0;

    public static final Slot0Configs PID =
        new Slot0Configs()
            .withKS(0.0)
            .withKG(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final InvertedValue GRABBER_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double GRABBER_STATOR_LIMIT = 0;
    public static final double GRABBER_SUPPLY_LIMIT = 0;

    public static final double GRABBER_GEAR_RATIO = 0;
  }

  public static final class Wrist {
    public static final String CANBUS = "";
    public static final int WRIST_ID = 0;
    public static final int ENCODER_ID = 0;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 0;
    public static final double SUPPLY_LIMIT = 0;

    public static final double SENSOR_TO_MECHANISM_GEAR_RATIO = 0;
    public static final double ROTOR_TO_MECHANISM_GEAR_RATIO = 0;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
    public static final double MOTION_MAGIC_ACCELERATION = 0.0;

    public static final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromRadians(0.0);
    public static final Rotation2d REVERSE_SOFT_LIMIT = Rotation2d.fromRadians(0.0);

    public static final Slot0Configs PID =
        new Slot0Configs().withKS(0.0).withKV(0.0).withKA(0.0).withKP(0.0).withKI(0.0).withKD(0.0);
  }
}
