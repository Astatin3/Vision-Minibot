package frc4388.robot.subsystems.differential;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

public class GyroSim implements GyroIO {
    private Supplier<Rotation2d> diffRot;

    public GyroSim(Supplier<Rotation2d> diffRot){
        this.diffRot = diffRot;
    }
    
    private Rotation2d offsetRot = new Rotation2d();

    @Override
    public void updateInputs(GyroState inputs) {
        inputs.connected = true;
        inputs.voltage = Volts.of(12);
        inputs.yaw = diffRot.get().rotateBy(offsetRot);
    }

    @Override
    public void reset() {
        offsetRot = diffRot.get().unaryMinus();
    }
}
