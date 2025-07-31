package frc4388.robot.subsystems.differential;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DiffReal implements DiffIO {
    TalonFX m_leftFront;
    TalonFX m_rightFront;
    TalonFX m_leftRear;
    TalonFX m_rightRear;
    PneumaticsControlModule m_pneumaticHub;

    DoubleSolenoid m_Solenoid;
    // DoubleSolenoid m_leftSolenoid;

    public DiffReal(TalonFX m_leftFront, TalonFX m_rightFront, TalonFX m_leftRear, TalonFX m_rightRear, PneumaticsControlModule pneumaticHub) {
        this.m_leftFront = m_leftFront;
        this.m_rightFront = m_rightFront;
        this.m_leftRear = m_leftRear;
        this.m_rightRear = m_rightRear;
        this.m_pneumaticHub = pneumaticHub;

        this.m_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        // this.m_rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);



        m_leftRear.setControl(new Follower(m_leftFront.getDeviceID(), false));
        m_rightRear.setControl(new Follower(m_rightFront.getDeviceID(), false));
        // m_rightRear.follow(m_rightFront);

        
        // this.m_rightFront.setInverted(true);
        // this.m_rightRear.setInverted(true);
    }

    @Override
    public void updateInputs(DiffState state) {
        // state.leftCurrent = new double[] {m_leftFront.getStatorCurrent(), m_leftRear.getStatorCurrent()};
        // state.leftOutputPosition = (m_leftFront.getSelectedSensorPosition() + m_leftRear.getSelectedSensorPosition()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;
        // state.leftOutputVelocity = (m_leftFront.getSelectedSensorVelocity() + m_leftRear.getSelectedSensorVelocity()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;

        // state.rightCurrent = new double[] {m_rightFront.getStatorCurrent(), m_rightRear.getStatorCurrent()};
        // state.rightOutputPosition = (m_rightFront.getSelectedSensorPosition() + m_rightFront.getSelectedSensorPosition()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;
        // state.rightOutputVelocity = (m_rightFront.getSelectedSensorVelocity() + m_rightFront.getSelectedSensorVelocity()) / 2 / DiffConstants.ENCODER_TICKS_PER_ROTATION;
    }
    
    @Override
    public void driveWithInput(double leftInput, double rightInput) {
        m_leftFront.set(leftInput);
        m_rightFront.set(rightInput);
    }

    @Override
    public void shiftUp() {
        m_Solenoid.set(Value.kForward);
        // m_rightSolenoid.set(Value.kForward);
    }

    @Override
    public void shiftDown() {
        m_Solenoid.set(Value.kReverse);
        // m_rightSolenoid.set(Value.kReverse);
    }
}
