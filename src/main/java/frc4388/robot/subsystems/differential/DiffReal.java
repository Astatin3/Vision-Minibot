package frc4388.robot.subsystems.differential;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class DiffReal implements DiffIO {
    TalonSRX m_leftFront;
    TalonSRX m_rightFront;
    TalonSRX m_leftRear;
    TalonSRX m_rightRear;

    public DiffReal(TalonSRX m_leftFront, TalonSRX m_rightFront, TalonSRX m_leftRear, TalonSRX m_rightRear) {
        this.m_leftFront = m_leftFront;
        this.m_rightFront = m_rightFront;
        this.m_leftRear = m_leftRear;
        this.m_rightRear = m_rightRear;


        m_leftRear.follow(m_leftFront);
        m_rightRear.follow(m_rightFront);

        
        this.m_rightFront.setInverted(true);
        this.m_rightRear.setInverted(true);
    }

    @Override
    public void updateInputs(DiffState state) {
        state.leftCurrent = new double[] {m_leftFront.getStatorCurrent(), m_leftRear.getStatorCurrent()};
        state.leftOutputPosition = (m_leftFront.getSelectedSensorPosition() + m_leftRear.getSelectedSensorPosition()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;
        state.leftOutputVelocity = (m_leftFront.getSelectedSensorVelocity() + m_leftRear.getSelectedSensorVelocity()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;

        state.rightCurrent = new double[] {m_rightFront.getStatorCurrent(), m_rightRear.getStatorCurrent()};
        state.rightOutputPosition = (m_rightFront.getSelectedSensorPosition() + m_rightFront.getSelectedSensorPosition()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;
        state.rightOutputVelocity = (m_rightFront.getSelectedSensorVelocity() + m_rightFront.getSelectedSensorVelocity()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;
    }
    
    @Override
    public void driveWithInput(double leftInput, double rightInput) {
        m_leftFront.set(TalonSRXControlMode.PercentOutput, leftInput);
        m_rightFront.set(TalonSRXControlMode.PercentOutput, rightInput);
    }
}
