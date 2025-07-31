package frc4388.robot.subsystems.differential;

import org.littletonrobotics.junction.AutoLog;

public interface DiffIO {
    @AutoLog
    public class DiffState {
        public double leftOutputPosition = 0;
        public double leftOutputVelocity = 0;
        public double[] leftCurrent = new double[] {};

        public double rightOutputPosition = 0;
        public double rightOutputVelocity = 0;
        public double[] rightCurrent = new double[] {};
    }

    public default void driveWithInput(double leftInput, double rightInput) {}

    public default void updateInputs(DiffState state) {}

    public default void shiftDown() {}
    public default void shiftUp() {}
}
