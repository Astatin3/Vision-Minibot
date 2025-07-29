package frc4388.robot.subsystems.differential;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc4388.utility.status.CanDevice;
import frc4388.utility.structs.Gains;

public class DiffConstants {
    
    public static final Distance WHEEL_RADIUS = Inches.of(2.5);
    public static final Distance WHEEL_RADIUS_TO_ARC = WHEEL_RADIUS.times(Math.PI * 2); //meters
    public static final Distance TRACK_DISPLACEMENT = Inches.of(6.5); //meters
    public static final Distance TRACK_WIDTH = TRACK_DISPLACEMENT.times(2);

    public static final int ENCODER_TICKS_PER_ROTATION = 1100;

    public static final Gains ROT_GAINS = new Gains(20, 0, 0);

    public static final class IDs {
        public static final CanDevice FRONT_LEFT_MOTOR = new CanDevice("SRX_FRONT_LEFT", 4);
        public static final CanDevice FRONT_RIGHT_MOTOR = new CanDevice("SRX_FRONT_RIGHT", 1);
        public static final CanDevice REAR_LEFT_MOTOR = new CanDevice("SRX_REAR_LEFT", 2);
        public static final CanDevice REAR_RIGHT_MOTOR = new CanDevice("SRX_REAR_RIGHT", 3);
        
        // public static final CanDevice FRONT_LEFT_MOTOR = new CanDevice("SRX_FRONT_LEFT", 3);
        // public static final CanDevice FRONT_RIGHT_MOTOR = new CanDevice("SRX_FRONT_RIGHT", 2);
        // public static final CanDevice REAR_LEFT_MOTOR = new CanDevice("SRX_REAR_LEFT", 1);
        // public static final CanDevice REAR_RIGHT_MOTOR = new CanDevice("SRX_REAR_RIGHT", 4);
    }
}
