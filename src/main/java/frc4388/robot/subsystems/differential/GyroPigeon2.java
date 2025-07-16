package frc4388.robot.subsystems.differential;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc4388.utility.status.Alerts;

public class GyroPigeon2 implements GyroIO {
    Pigeon2 m_gyro;

    StatusSignal<Voltage> voltage;

    public GyroPigeon2(Pigeon2 m_gyro){
        this.m_gyro = m_gyro;

        voltage = m_gyro.getSupplyVoltage();
    }

    @Override
    public void updateInputs(GyroState state) {
        voltage.refresh();
        
        state.connected = m_gyro.isConnected();
        state.voltage = voltage.getValue();
        state.yaw = m_gyro.getRotation2d();

        Alerts.validate(!state.connected, "Gyro is not connected!", AlertType.kError);
    }

    @Override
    public void reset() {
        m_gyro.reset();
    }
}
