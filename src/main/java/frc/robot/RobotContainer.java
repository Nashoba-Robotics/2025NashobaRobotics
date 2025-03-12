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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManualExtensionCommand;
import frc.robot.commands.test.ClimberTestDownCommand;
import frc.robot.commands.test.ClimberTestUpCommand;
import frc.robot.commands.test.ElevatorDutyCycleCommand;
import frc.robot.commands.test.TuneClimberCommand;
import frc.robot.commands.test.TuneElevatorCommand;
import frc.robot.commands.test.TuneWristCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Manipulator manipulator;
  private final Hopper hopper;
  private final Climber climber;
  private final LEDSubsystem leds;

  public final Superstructure superstructure;

  // // Controller
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  public static final CommandXboxController testController = new CommandXboxController(2);

  //   // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevator = new Elevator();
    wrist = new Wrist();
    manipulator = new Manipulator();
    hopper = new Hopper();
    climber = new Climber();

    superstructure = new Superstructure(elevator, wrist, manipulator, hopper, climber);

    leds = new LEDSubsystem(superstructure);

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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    NamedCommands.registerCommand("L4Prep", superstructure.autoSetL4Coral());
    NamedCommands.registerCommand("L4Score", manipulator.L4ejectCommand());
    NamedCommands.registerCommand("L2Prep", superstructure.autoSetL2Coral());
    NamedCommands.registerCommand("L2Score", manipulator.ejectCommand());
    NamedCommands.registerCommand("SetIntake", superstructure.setIntake());
    NamedCommands.registerCommand("SetNeutral", superstructure.setNeutral());
    NamedCommands.registerCommand("UnendingIntake", superstructure.autoIntake());
    NamedCommands.registerCommand(
        "AutoDrive",
        DriveCommands.driveToPose(
                drive, () -> drive.getPose().nearest(Arrays.asList(scoringPositions)))
            .until(
                () ->
                    Math.abs(
                                drive.getPose().nearest(Arrays.asList(scoringPositions)).getX()
                                    - drive.getPose().getX())
                            <= 0.025
                        && Math.abs(
                                drive.getPose().nearest(Arrays.asList(scoringPositions)).getY()
                                    - drive.getPose().getY())
                            <= 0.025
                        && Math.abs(
                                drive
                                        .getPose()
                                        .nearest(Arrays.asList(scoringPositions))
                                        .getRotation()
                                        .getRadians()
                                    - drive.getPose().getRotation().getRadians())
                            <= 0.15)
            .withTimeout(0.4));

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive a foot", new PathPlannerAuto("Taxi"));
    autoChooser.addOption("Old3Piece Right", new PathPlannerAuto("Old3Piece", false));
    autoChooser.addOption("Old3Piece Left", new PathPlannerAuto("Old3Piece", true));
    autoChooser.addOption("New3Piece Right", new PathPlannerAuto("New3Piece", false));
    autoChooser.addOption("New3Piece Left", new PathPlannerAuto("New3Piece", true));
    autoChooser.addOption("TuneAuto", new PathPlannerAuto("TuneDrive", true));

    // Configure the button bindings
    configureButtonBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Logger.recordOutput("AutoAlignGoals", scoringPositions);

    SmartDashboard.putData(new TuneElevatorCommand(elevator));
    SmartDashboard.putData(new ElevatorDutyCycleCommand(elevator));
    SmartDashboard.putData(new TuneWristCommand(wrist));
    SmartDashboard.putData(new ManualExtensionCommand(operator, elevator, wrist));
    SmartDashboard.putData(new TuneClimberCommand(climber));

    testController.a().onTrue(superstructure.setL2Coral());
    testController.b().onTrue(superstructure.setL3Coral());

    operator.rightBumper().whileTrue(manipulator.slowSpitCommand());
    operator.leftBumper().whileTrue(manipulator.slowIntakeCommand());

    operator.a().onTrue(climber.deployClimber());
    operator
        .rightTrigger(0.15)
        .and(operator.y())
        .onTrue(new ClimberTestUpCommand(climber, operator));
    operator
        .leftTrigger(0.15)
        .and(operator.y())
        .onTrue(new ClimberTestDownCommand(climber, operator));

    driver
        .y()
        .whileTrue(
            DriveCommands.driveToPose(
                    drive, () -> drive.getPose().nearest(Arrays.asList(scoringPositions)))
                .alongWith(
                    Commands.waitUntil(
                            () ->
                                Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getX()
                                                - drive.getPose().getX())
                                        <= 0.03
                                    && Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getY()
                                                - drive.getPose().getY())
                                        <= 0.03
                                    && Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getRotation()
                                                    .getRadians()
                                                - drive.getPose().getRotation().getRadians())
                                        <= 0.15)
                        .andThen(superstructure.setL4Coral())));
    driver
        .b()
        .whileTrue(
            DriveCommands.driveToPose(
                    drive, () -> drive.getPose().nearest(Arrays.asList(scoringPositions)))
                .alongWith(
                    Commands.waitUntil(
                            () ->
                                Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getX()
                                                - drive.getPose().getX())
                                        <= 0.03
                                    && Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getY()
                                                - drive.getPose().getY())
                                        <= 0.03
                                    && Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getRotation()
                                                    .getRadians()
                                                - drive.getPose().getRotation().getRadians())
                                        <= 0.15)
                        .andThen(superstructure.setL3Coral())));
    driver
        .a()
        .whileTrue(
            DriveCommands.driveToPose(
                    drive, () -> drive.getPose().nearest(Arrays.asList(scoringPositions)))
                .alongWith(
                    Commands.waitUntil(
                            () ->
                                Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getX()
                                                - drive.getPose().getX())
                                        <= 0.03
                                    && Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getY()
                                                - drive.getPose().getY())
                                        <= 0.03
                                    && Math.abs(
                                            drive
                                                    .getPose()
                                                    .nearest(Arrays.asList(scoringPositions))
                                                    .getRotation()
                                                    .getRadians()
                                                - drive.getPose().getRotation().getRadians())
                                        <= 0.15)
                        .andThen(superstructure.setL2Coral())));

    driver.rightBumper().onTrue(superstructure.setL1Coral());
    driver.rightStick().onTrue(manipulator.spitCommand());

    driver.povUp().onTrue(superstructure.setBargeAlgae());
    driver.povRight().onTrue(superstructure.setL3Algae());
    driver.povDown().onTrue(superstructure.setL2Algae());
    driver.povLeft().onTrue(superstructure.setProcessor());

    driver.leftTrigger(0.65).whileTrue(superstructure.setIntake());
    driver.leftTrigger(0.65).onFalse(superstructure.setNeutral());
    driver.leftBumper().onTrue(superstructure.setNeutral());

    // // // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // // // Switch to X pattern when start button is pressed
    // driver.start().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver.start().onTrue(manipulator.ejectCommand());

    // // Reset gyro to 0° when B button is pressed
    driver
        .back()
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
