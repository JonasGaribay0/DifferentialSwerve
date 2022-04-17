// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Swerve.*;

public class SwerveDrive extends SubsystemBase {

  private SwerveDriveKinematics swerveDriveKinematics;
  private SwerveModule[] swerveModules = new SwerveModule[MODULE_AMOUNT];
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[MODULE_AMOUNT];

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    swerveDriveKinematics = new SwerveDriveKinematics(
      FRONT_LEFT_MODULE_POSITION,
      FRONT_RIGHT_MODULE_POSITION,
      BACK_LEFT_MODULE_POSITION,
      BACK_RIGHT_MODULE_POSITION
    );

    for (int i = 0; i < MODULE_AMOUNT; i++) {
      swerveModules[i] = new SwerveModule(i);    
    }
    for (int i = 0; i < MODULE_AMOUNT; i++) {
      swerveModuleStates[i] = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }

  }

  /**
   * 
   * @param xVelocity
   * @param yVelocity
   * @param angularVelocity
   */
  public void drive(double xVelocity, double yVelocity, double angularVelocity) {
    ChassisSpeeds input = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);

    swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(input);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_SPEED);
    drive(swerveModuleStates);
  }

  public void drive(SwerveModuleState[] states) {
    swerveModuleStates = states;
    for (int i = 0; i < MODULE_AMOUNT; i++) {
      swerveModules[i].drive(states[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
