// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.Logitech;

import static frc.robot.util.Logitech.Ports.*;
import static frc.robot.Constants.Joystick.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDrive swerveDrive = new SwerveDrive();

  private Logitech dStick = new Logitech(Driver.PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDefaultCommands();

    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    swerveDrive.setDefaultCommand(
      new RunCommand(
        () -> {
          // get inputs then square them, preserving sign
          double fwd = dStick.getRawAxis(LEFT_STICK_Y);
          double str = dStick.getRawAxis(LEFT_STICK_X);
          double rot = dStick.getRawAxis(RIGHT_STICK_X);
          
          fwd = -Math.signum(fwd) * fwd * fwd * Constants.Swerve.MAX_WHEEL_SPEED;
          str = -Math.signum(str) * str * str * Constants.Swerve.MAX_WHEEL_SPEED;
          rot = -Math.signum(rot) * rot * rot * Constants.Swerve.MAX_ANGULAR_SPEED;

          // pass inputs into drivetrain
          swerveDrive.drive(fwd, str, rot);

        }, 
        swerveDrive
      )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    dStick.setDeadband(LEFT_STICK_X, Driver.LEFT_X_DEADBAND);
    dStick.setDeadband(LEFT_STICK_Y, Driver.LEFT_Y_DEADBAND);
    dStick.setDeadband(RIGHT_STICK_X, Driver.RIGHT_X_DEADBAND);
    dStick.setDeadband(LEFT_TRIGGER, Driver.LEFT_TRIGGER_DEADBAND);
    dStick.setDeadband(RIGHT_TRIGGER, Driver.RIGHT_TRIGGER_DEADBAND);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
