package frc4388.utility.structs;

import edu.wpi.first.math.geometry.Translation2d;

// Abstract 
public interface Drivebase {

    // Swerve drive - Drive relative to field.
    public default void driveFieldRelative(Translation2d left, Translation2d right) {}
    
    // Swerve drive - Drive relative to robot;
    public default void driveRobotOriented(Translation2d left, Translation2d right) {}
    
    // Diff drive - Arcade drive
    // Left stick forward and back is forward and back movement
    // Right stick left and right is turning
    public default void arcadeDrive(Translation2d left, Translation2d right) {}

    public void resetOdometry();
}
