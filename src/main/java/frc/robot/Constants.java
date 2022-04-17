// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double METERS_PER_INCH = 0.0254;

    public static final class Swerve {

        public static final int[] MOTOR_1_PORTS = {0, 0, 0, 0};
        public static final int[] MOTOR_2_PORTS = {0, 0, 0, 0};

        public static final boolean[] MOTOR_1_INVERTS = {false, false, false, false};
        public static final boolean[] MOTOR_2_INVERTS = {false, false, false, false};

        public static final int MODULE_AMOUNT = 4;

        public static final int ANGLE_ENCODER_PORT = 0;
        public static final double ANGLE_ENCODER_CPR = 0.0;

        public static final double NEO_ENCODER_CPR = 42.0; // TODO: I dont know

        public static final double SPEED_PID_KP = 0.0;
        public static final double SPEED_PID_KI = 0.0;
        public static final double SPEED_PID_KD = 0.0;

        public static final double ANGLE_PID_KP = 0.0;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double ANGLE_PID_TOLERANCE = 0.0;
        public static final double ANGLE_PID_MAX_ACCELERATION = 0.0; // In encoder ticks per second per second
        public static final double ANGLE_PID_MAX_VELOCITY = ANGLE_PID_MAX_ACCELERATION * Math.sqrt((ANGLE_ENCODER_CPR / 4) / ANGLE_PID_MAX_ACCELERATION); // in encoder ticks per second

        public static final double DRIVE_GEAR_RATIO = 0.0;
        public static final double MOTOR_GEAR_TEETH = 0.0;
        public static final double CARRIER_GEAR_TEETH = 0.0;

        // In meters
        public static final double WHEEL_BASE = 23.111 * METERS_PER_INCH;
        public static final double TRACK_WIDTH = 23.111 * METERS_PER_INCH; // TODO: good?
        public static final double WHEEL_DIAMETER = 4.0 * METERS_PER_INCH;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // translation 2d considers the front of the robot as the positive x direction
        // and the left of the robot as the positive y direction
        public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        public static final double MAX_WHEEL_SPEED = 5.2;
        public static final double MAX_ANGULAR_SPEED = 6.28;         

    }

    public static final class Joystick {
        public static final class Driver {
            public static final int PORT = 0;
            public static final double LEFT_X_DEADBAND = 0.1;
            public static final double LEFT_Y_DEADBAND = 0.1;
            public static final double RIGHT_X_DEADBAND = 0.1;
            public static final double LEFT_TRIGGER_DEADBAND = 0.5;
            public static final double RIGHT_TRIGGER_DEADBAND = 0.5;
        }

        public static class Manipulator {
            public static final int PORT = 1;
            public static final double LEFT_TRIGGER_DEADBAND = 0.5;
            public static final double RIGHT_TRIGGER_DEADBAND = 0.5;
            public static final double LEFT_STICK_Y_DEADBAND = 0.1;
            public static final double RIGHT_STICK_Y_DEADBAND = 0.1;
        }
    }

}
