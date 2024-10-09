package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Declares control key bindings
 */
public class Controls {
    
    /* Controllers */
    public static final Joystick driverController = new Joystick(0);
    // public static final XboxController upController = new XboxController(1);
    public static final CommandXboxController upController = new CommandXboxController(1);

     /* Driver Controls */
    public static final int translationAxis = XboxController.Axis.kLeftY.value;
    public static final int strafeAxis = XboxController.Axis.kLeftX.value;
    public static final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    public static final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
    public static final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

    public static final JoystickButton dampen = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    public static final POVButton up = new POVButton(driverController, 90);
    public static final POVButton down = new POVButton(driverController, 270);
    public static final POVButton right = new POVButton(driverController, 180);
    public static final POVButton left = new POVButton(driverController, 0);

}
