package frc4388.robot.subsystems.differential;

import static edu.wpi.first.units.Units.Meters;

import java.time.Duration;
import java.time.Instant;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class DiffSim implements DiffIO {
    private static final DCMotor MOTOR_SIM = new DCMotor(
        12.3, 
        3, 
        -1.325, 
        -0.5, 
        -5.5, 
        2);

    public DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(MOTOR_SIM, 
        1 / (double) DiffConstants.ENCODER_TICKS_PER_ROTATION, 
        1, 
        1, 
        DiffConstants.WHEEL_RADIUS.in(Meters),
        DiffConstants.TRACK_WIDTH.in(Meters), 
        null
    );

    private Supplier<Rotation2d> currentRot = new Supplier<Rotation2d>() {
        @Override
        public Rotation2d get() {
            return sim.getHeading();
        }
    };

    private double clamp(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }
    
    @Override
    public void driveWithInput(double leftInput, double rightInput) {
        sim.setInputs(
            clamp(leftInput, -1, 1) * 12, 
            clamp(rightInput, -1, 1) * 12
        );
    }

    Instant lastUpdate = Instant.now();

    private double getTiemDelta() {
        Instant now = Instant.now();
        double elapsedSeconds = ((double) Duration.between(lastUpdate, now).toMillis()) / 1000;
        // System.out.println(elapsedSeconds);%
        lastUpdate = now;
        return elapsedSeconds;
    }

    @Override
    public void updateInputs(DiffState state) {
        sim.update(getTiemDelta());

        state.leftOutputPosition =  sim.getLeftPositionMeters() / DiffConstants.WHEEL_RADIUS_TO_ARC.in(Meters);
        state.leftOutputVelocity = sim.getLeftVelocityMetersPerSecond() / DiffConstants.WHEEL_RADIUS_TO_ARC.in(Meters);
        state.leftCurrent = new double[]{sim.getLeftCurrentDrawAmps()};

        state.rightOutputPosition =  sim.getRightPositionMeters() / DiffConstants.WHEEL_RADIUS_TO_ARC.in(Meters);
        state.rightOutputVelocity = sim.getRightVelocityMetersPerSecond() / DiffConstants.WHEEL_RADIUS_TO_ARC.in(Meters);
        state.rightCurrent = new double[]{sim.getRightCurrentDrawAmps()};
    }

    public Supplier<Rotation2d> getSimAngle() {
        return currentRot;
    }
}
