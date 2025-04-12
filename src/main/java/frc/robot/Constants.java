package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    public static final String CANBUS = "rio";
    public static final int LEADER_ID = 8;
    public static final int FOLLOWER_ID = 9;

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 80;
    public static final double SUPPLY_LIMIT = 80;

    public static final double GEAR_RATIO = 3; // Motor to output on elevator is 3:1
    public static final double PULLY_RAIDUS = (0.0175133); // meters

    public static final double MOTION_MAGIC_CRUISE_VELOCITY =
        Units.radiansToRotations(3 / PULLY_RAIDUS);
    public static final double MOTION_MAGIC_ACCELERATION =
        Units.radiansToRotations(20 / PULLY_RAIDUS);

    public static final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromRadians(81);
    public static final double ELEVATOR_TOLERANCE = 0.03; // meters

    public static final Slot0Configs PID =
        new Slot0Configs()
            .withKS(0)
            .withKG(0.4 / 12)
            .withKV(0.0375)
            .withKA(0.0)
            .withKP(1.75)
            .withKI(0.0)
            .withKD(0.025)
            .withGravityType(GravityTypeValue.Elevator_Static);
  }

  public static final class Manipulator {
    public static final String CANBUS = "rio";
    public static final int MANIPULATOR_ID = 12;
    public static final int SENSOR_ID = 0;

    public static final double SENSOR_DISTANCE_THRESHOLD = 125;

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 90;
    public static final double SUPPLY_LIMIT = 90;

    public static final double GEAR_RATIO = 4;
  }

  public static final class Hopper {
    public static final String CANBUS = "rio";
    public static final int HOPPER_ID = 13;

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 30;
    public static final double SUPPLY_LIMIT = 30;

    public static final double GEAR_RATIO = 0;
  }

  public static final class Climber {
    public static final String CANBUS = "CANivore";
    public static final int LEADER_ID = 14;
    public static final int FOLLOWER_ID = 15;

    public static final InvertedValue PIVOT_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double PIVOT_STATOR_LIMIT = 100;
    public static final double PIVOT_SUPPLY_LIMIT = 100;

    public static final double PIVOT_GEAR_RATIO = 206.18181818;

    public static final double PIVOT_MOTION_MAGIC_CRUISE_VELOCITY = 10000.0;
    public static final double PIVOT_MOTION_MAGIC_ACCELERATION = 10000.0;

    public static final Slot0Configs PID =
        new Slot0Configs()
            .withKS(0.0)
            .withKG(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(10.0)
            .withKI(0.0)
            .withKD(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine);
  }

  public static final class Wrist {
    public static final String CANBUS = "rio";
    public static final int WRIST_ID = 11;

    public static final int ENCODER_ID = 4;

    public static final SensorDirectionValue ENCODER_INVERTED =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double ENCODER_OFFSET_RADIANS = 4.1156704539;

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double STATOR_LIMIT = 45;
    public static final double SUPPLY_LIMIT = 45;

    public static final double SENSOR_TO_MECHANISM_GEAR_RATIO = 1;
    public static final double ROTOR_TO_MECHANISM_GEAR_RATIO = 63.21;

    public static final Rotation2d MOTION_MAGIC_CRUISE_VELOCITY = Rotation2d.fromRadians(625);
    public static final Rotation2d MOTION_MAGIC_ACCELERATION = Rotation2d.fromRadians(625);

    public static final Rotation2d WRIST_TOLERANCE = Rotation2d.fromRadians(0.05);
    public static final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromRadians(0.0);
    public static final Rotation2d REVERSE_SOFT_LIMIT = Rotation2d.fromRadians(0.0);

    public static final Slot0Configs PID =
        new Slot0Configs()
            .withKS(0.15 / 12)
            .withKG(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(7.5)
            .withKI(0.0)
            .withKD(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  }
}
