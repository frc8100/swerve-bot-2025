package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    // private final XboxController upController = new XboxController(1);
    private final CommandXboxController upController = new CommandXboxController(1);

    /* Driver Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final LauncherSubsystem m_launcher = new LauncherSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> 1 //speed multiplier
            )
        );

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
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        // Heading lock bindings
        up
            .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d90))
            .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        left
            .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d180))
            .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        right
            .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d0))
            .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        down
            .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d270))
            .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));

        // Up system bindings

        // Note: on the Logitech controller, x and a are swapped
        upController.b().whileTrue(m_intake.intake());
        upController.x().whileTrue(m_intake.eintake());
        upController.rightBumper().whileTrue(m_arm.up());
        upController.leftBumper().whileTrue(m_arm.down());
        // launcher controls (button to pre-spin the launcher and button to launch)
        // upController.leftTrigger().whileTrue(m_launcher.pushOut());
        // upController.rightTrigger().whileTrue(m_launcher.pushIn());

        upController.a().whileTrue(m_launcher.pushOut());
        upController.y().whileTrue(m_launcher.pushIn());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
