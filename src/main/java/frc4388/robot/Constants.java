/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

// import com.pathplanner.l;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveDriveConstants {

        public static final double MAX_ROT_SPEED        = 3.5;
        public static final double AUTO_MAX_ROT_SPEED = 1.5;
        public static final double MIN_ROT_SPEED        = 1.0;
        public static       double ROTATION_SPEED       = MAX_ROT_SPEED;
        public static       double PLAYBACK_ROTATION_SPEED = AUTO_MAX_ROT_SPEED;
        public static       double ROT_CORRECTION_SPEED = 10; // MIN_ROT_SPEED;

        public static final String CANBUS_NAME = "IDK";
    
        public static final double CORRECTION_MIN = 10;
        public static final double CORRECTION_MAX = 50;
        
        public static final double[] GEARS = {0.25, 0.5, 1.0};

        public static final double SLOW_SPEED = 0.25;
        public static final double FAST_SPEED = 0.5;
        public static final double TURBO_SPEED = 1.0;
    
        public static final class DefaultSwerveRotOffsets {
          public static final double FRONT_LEFT_ROT_OFFSET  = 0.0; //TODO: per robot swerve module offsets.
          public static final double FRONT_RIGHT_ROT_OFFSET = 0.0; //TODO: per robot swerve module offsets.
          public static final double BACK_LEFT_ROT_OFFSET   = 0.0; //TODO: per robot swerve module offsets.
          public static final double BACK_RIGHT_ROT_OFFSET  = 0.0; //TODO: per robot swerve module offsets.
        }

        public static final class IDs {
            public static final int FR_ID = 22;
            public static final int FL_ID = 20;
            public static final int BL_ID = 21;
            public static final int BR_ID = 23;
            // public static final int RIGHT_FRONT_WHEEL_ID = 2;
            // public static final int RIGHT_FRONT_STEER_ID = 3;
            // public static final int RIGHT_FRONT_ENCODER_ID = 10;
            
            // public static final int LEFT_FRONT_WHEEL_ID = 4;
            // public static final int LEFT_FRONT_STEER_ID = 5;
            // public static final int LEFT_FRONT_ENCODER_ID = 11;
            
            // public static final int LEFT_BACK_WHEEL_ID = 6;
            // public static final int LEFT_BACK_STEER_ID = 7;
            // public static final int LEFT_BACK_ENCODER_ID = 12;
            
            // public static final int RIGHT_BACK_WHEEL_ID = 8;  
            // public static final int RIGHT_BACK_STEER_ID = 9;
            // public static final int RIGHT_BACK_ENCODER_ID = 13;

            public static final int DRIVE_PIGEON_ID = 10;
        }
    
        public static final class PIDConstants {
            public static final int SWERVE_SLOT_IDX = 0;
            public static final int SWERVE_PID_LOOP_IDX = 1;
            public static final Gains SWERVE_GAINS = new Gains(50, 0.0, 0.32, 0.0, 0, 0.0);

            public static final Gains TEST_SWERVE_GAINS = new Gains(1.2, 0.0, 0.0, 0.0, 0, 0.0);

        }
    
        public static final class AutoConstants {
            public static final Gains X_CONTROLLER = new Gains(0.8, 0.0, 0.0);
            public static final Gains Y_CONTROLLER = new Gains(0.8, 0.0, 0.0);
            public static final Gains THETA_CONTROLLER = new Gains(-0.8, 0.0, 0.0);
            public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI/2, Math.PI/2); // TODO: tune
            
            public static final double PATH_MAX_VEL = 0.3; // TODO: find the actual value
            public static final double PATH_MAX_ACC = 0.3; // TODO: find the actual value

            public static final double MASS = 10; // KG
            public static final double MOI = 10;  // Moment of inertia

            public static final double WHEEL_RADIUS = 0.05; // Meters
            public static final double MAX_DRIVE_VELOCITY = 0.5; // Meters per second
            public static final double MAX_ROT_SPEED = 0.2; // Rotations per second


            public static final double COEFFICENT_OF_FRICTION = 1.0; // Between 0 and 1
            public static final DCMotor TALON_SRX_MOTOR = DCMotor.getVex775Pro(1); //TODO: Get actual motor constants
            public static final double DRIVE_CURRENT_LIMIT = 100000; //TODO: Get actual value


            public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
                WHEEL_RADIUS, 
                MAX_DRIVE_VELOCITY, 
                COEFFICENT_OF_FRICTION, 
                TALON_SRX_MOTOR,
                DRIVE_CURRENT_LIMIT, 
                2);
 
            public static final RobotConfig PP_ROBOT_CONFIG = new RobotConfig(
                MASS, 
                MOI, 
                MODULE_CONFIG,
                new Translation2d[] {
                    new Translation2d(),
                    new Translation2d(),
                    new Translation2d(),
                    new Translation2d(),
                });

                
            public static final double ROBOT_LOOP_TIME = 0.02; // Time it takes for the robot to run a loop
            public static final PathFollowingController PP_PATH_FOLLOWING_CONTROLLER = new PPLTVController(ROBOT_LOOP_TIME);
        }
    
        public static final class Conversions {
            public static final double JOYSTICK_TO_METERS_PER_SECOND_FAST = 6.22;
            public static final double JOYSTICK_TO_METERS_PER_SECOND_SLOW = JOYSTICK_TO_METERS_PER_SECOND_FAST * 0.5;
        
            public static final double MOTOR_REV_PER_WHEEL_REV = 5.12;
            public static final double MOTOR_REV_PER_STEER_REV = 12.8;
        
            public static final double TICKS_PER_MOTOR_REV = 0.5;
            public static final double WHEEL_DIAMETER_INCHES = 3.9;
            public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
        
            public static final double WHEEL_REV_PER_MOTOR_REV = 1 / MOTOR_REV_PER_WHEEL_REV;
            public static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REV_PER_WHEEL_REV;
            public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REV / INCHES_PER_WHEEL_REV;
            public static final double INCHES_PER_TICK = 1 / TICKS_PER_INCH;
        
            public static final double TICK_TIME_TO_SECONDS = 10;
            public static final double SECONDS_TO_TICK_TIME = 1 / TICK_TIME_TO_SECONDS;
        }
    
        public static final class Configurations {
            public static final double OPEN_LOOP_RAMP_RATE = 0.2;
            public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
            public static final double NEUTRAL_DEADBAND = 0.04;
        }
    
        public static final double MAX_SPEED_FEET_PER_SECOND = 20.4;
        public static final double MAX_ANGULAR_SPEED_FEET_PER_SECOND = 2 * 2 * Math.PI;
    
        // dimensions
        public static final double WIDTH = 18.5;
        public static final double HEIGHT = 18.5;
        public static final double HALF_WIDTH = WIDTH / 2.d;
        public static final double HALF_HEIGHT = HEIGHT / 2.d;
    
        // misc
        public static final int TIMEOUT_MS = 30;
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
      }
    
    public static final class VisionConstants {
        public static final String CAMERA_NAME = "Camera_Module_v1";
    }

    public static final class DriveConstants {
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
    }
    
    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 0;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
        public static final int XBOX_PROGRAMMER_ID = 2;
        public static final double LEFT_AXIS_DEADBAND = 0.20;

    }
}
