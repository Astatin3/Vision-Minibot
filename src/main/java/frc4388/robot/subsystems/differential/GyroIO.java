package frc4388.robot.subsystems.differential;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

public interface GyroIO {
    @AutoLog
    public class GyroState {
        public boolean connected = false;
        public Voltage voltage;
        public Rotation2d yaw;
    }
    
    public default void updateInputs(GyroState inputs) {}

    public default void reset() {}
}
