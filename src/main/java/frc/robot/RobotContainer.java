package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManualExtensionCommand;
import frc.robot.commands.test.ElevatorDutyCycleCommand;
import frc.robot.commands.test.TuneElevatorCommand;
import frc.robot.commands.test.TuneWristCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  //   private final Vision vision;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Manipulator manipulator;

  private final Superstructure superstructure;

  // // Controller
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  private Trigger algae = operator.rightTrigger(0.65);
  private Trigger coral = operator.leftTrigger(0.65);
  private Trigger prepHeight = driver.rightBumper();

  //   // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    elevator = new Elevator();
    wrist = new Wrist();
    manipulator = new Manipulator();

    superstructure = new Superstructure(elevator, wrist);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    NamedCommands.registerCommand("L2Prep", superstructure.scoreL2Coral());
    NamedCommands.registerCommand("CoralScore", manipulator.ejectCommand());
    NamedCommands.registerCommand("SetIntake", superstructure.setIntake());
    NamedCommands.registerCommand("Intake", manipulator.intakeCommand());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Left side Score 1", new PathPlannerAuto("Score 1", true));
    autoChooser.addOption("Right side Score 1", new PathPlannerAuto("Score 1", false));
    autoChooser.addOption("Left side Score 2", new PathPlannerAuto("Score 2", true));
    autoChooser.addOption("Right side Score 2", new PathPlannerAuto("Score 2", false));
    autoChooser.addOption("Drive a foot", new PathPlannerAuto("Taxi"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    SmartDashboard.putData(new TuneElevatorCommand(elevator));
    SmartDashboard.putData(new ElevatorDutyCycleCommand(elevator));
    SmartDashboard.putData(new TuneWristCommand(wrist));
    SmartDashboard.putData(new ManualExtensionCommand(operator, elevator, wrist));

    driver.leftTrigger(0.65).onTrue(superstructure.setIntake());
    driver.leftTrigger(0.65).whileTrue(manipulator.intakeCommand());
    driver.rightTrigger(0.65).onTrue(manipulator.ejectCommand());
    driver.leftBumper().onTrue(superstructure.setNeutral());
    driver.y().onTrue(manipulator.L1ejectCommand());

    algae
        .and(prepHeight)
        .and(operator.a())
        .onTrue(
            superstructure
                .setL2Algae()
                .andThen(
                    manipulator
                        .intakeCommand()
                        .alongWith(new ManualExtensionCommand(operator, elevator, wrist))));
    algae
        .and(prepHeight)
        .and(operator.b())
        .onTrue(
            superstructure
                .setL3Algae()
                .andThen(
                    manipulator
                        .intakeCommand()
                        .alongWith(new ManualExtensionCommand(operator, elevator, wrist))));
    algae
        .and(prepHeight)
        .and(operator.y())
        .onTrue(
            superstructure
                .setBargeAlgae()
                .andThen(
                    manipulator
                        .intakeCommand()
                        .alongWith(new ManualExtensionCommand(operator, elevator, wrist))));

    coral
        .and(prepHeight)
        .and(operator.a())
        .onTrue(
            superstructure
                .scoreL2Coral()
                .andThen(new ManualExtensionCommand(operator, elevator, wrist)));
    coral
        .and(prepHeight)
        .and(operator.b())
        .onTrue(
            superstructure
                .scoreL3Coral()
                .andThen(new ManualExtensionCommand(operator, elevator, wrist)));
    coral
        .and(prepHeight)
        .and(operator.y())
        .onTrue(
            superstructure
                .scoreL4Coral()
                .andThen(new ManualExtensionCommand(operator, elevator, wrist)));
    coral
        .and(prepHeight)
        .and(operator.x())
        .onTrue(
            superstructure
                .scoreL1Coral()
                .andThen(new ManualExtensionCommand(operator, elevator, wrist)));

    // // // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // // Switch to X pattern when start button is pressed
    driver.start().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                new Rotation2d(
                                    DriverStation.getAlliance().orElse(Alliance.Blue)
                                            == Alliance.Blue
                                        ? 0
                                        : Math.PI))),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
