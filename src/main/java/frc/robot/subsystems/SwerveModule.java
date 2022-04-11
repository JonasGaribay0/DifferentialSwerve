// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;


import static frc.robot.Constants.Swerve.*;

public class SwerveModule extends SubsystemBase {

  private CANSparkMax motor1;
  private CANSparkMax motor2;
  private AnalogInput angleEncoder;

  private PIDController motor1PID;
  private PIDController motor2PID;

  private ProfiledPIDController anglePID;


  /** Creates a new SwerveModule. */
  public SwerveModule() {
    motor1 = new CANSparkMax(MOTOR_1_PORT, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();
    motor1.setIdleMode(IdleMode.kBrake);
    motor2 = new CANSparkMax(MOTOR_2_PORT, MotorType.kBrushless);
    motor2.restoreFactoryDefaults();
    motor2.setIdleMode(IdleMode.kBrake);

    anglePID = new ProfiledPIDController(
      ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, 
      new TrapezoidProfile.Constraints(ANGLE_PID_MAX_VELOCITY, ANGLE_PID_MAX_ACCELERATION)
    );
    anglePID.enableContinuousInput(0.0, ANGLE_ENCODER_CPR);
    anglePID.setTolerance(ANGLE_PID_TOLERANCE);

    angleEncoder = new AnalogInput(ANGLE_ENCODER_PORT);

  }

  public void drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, new Rotation2d(nativeToRadians(getAngle())));
    
    double angleSetpointNative = radiansToNative(state.angle.getRadians());
    double anglePIDOutput = anglePID.calculate(getAngle(), angleSetpointNative);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond);
    double motor1PIDOutput = motor1PID.calculate(getSpeed(), speedSetpointNative);
    double motor2PIDOutput = motor2PID.calculate(getSpeed(), speedSetpointNative);

    motor1.setVoltage(
      motor1PIDOutput +
      state.speedMetersPerSecond == 0.0 ? 0.0 : anglePIDOutput
    );
    motor2.setVoltage(
      -motor2PIDOutput +
      state.speedMetersPerSecond == 0.0 ? 0.0 : anglePIDOutput
    );

  }


    /**
   * 
   * @return meters per count of talon fx encoder based on current gear
   */
  public double getMetersPerCount() {
    return WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO / NEO_ENCODER_CPR;
  }



  /**
   * 
   * @param metersPerSecond the speed in meters per second to be converted
   * @return the equivalent speed motor encoder velocity in counts per 100 ms
   */
  public double metersPerSecondToNative(double metersPerSecond) {
    return metersPerSecond / getMetersPerCount() / 10;
  }

  

  /**
   * 
   * @param encoderVelocity the velocity to be converted in encoder counts per decisecond
   * @return the converted speed of the wheel in meters per second
   */
  public double nativeToMetersPerSecond(double encoderVelocity) {
    return encoderVelocity * 10 * getMetersPerCount();
  }

    /**
   * @param encoderCounts number of encoder counts to convert
   * @return absolute encoder counts converted to radians
   */
  public static double nativeToRadians(double encoderCounts) {
    return encoderCounts * 2 * Math.PI / ANGLE_ENCODER_CPR;
  }


  /**
   * 
   * @param radians number of radians to convert
   * @return radians converted to absolute encoder counts
   */
  public static double radiansToNative(double radians) {
    return radians / (2 * Math.PI) * ANGLE_ENCODER_CPR;
  }

  public double getAngle() {
    return -1 * MathUtil.inputModulus(angleEncoder.getAverageVoltage() , 0, ANGLE_ENCODER_CPR) + ANGLE_ENCODER_CPR;
  }

  public double getSpeed() {
    return nativeToMetersPerSecond(
      motor1.getEncoder().getVelocity() + 
      Math.abs(motor2.getEncoder().getVelocity())
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
