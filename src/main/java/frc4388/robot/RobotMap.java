/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc4388.robot.subsystems.differential.DiffConstants;
import frc4388.robot.subsystems.differential.DiffIO;
import frc4388.robot.subsystems.differential.DiffReal;
import frc4388.robot.subsystems.differential.DiffSim;
import frc4388.robot.subsystems.differential.GyroIO;
import frc4388.robot.subsystems.differential.GyroPigeon2;
import frc4388.robot.subsystems.differential.GyroSim;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
    // private Pigeon2 m_pigeon2 = new Pigeon2(SwerveDriveConstants.IDs.DRIVE_PIGEON.id);
    // public RobotGyro gyro = new RobotGyro(m_pigeon2);

    public DiffIO m_DiffDrive;
    // public GyroIO m_gyro;

    public static RobotMap configureReal() {
        RobotMap map = new RobotMap();

        TalonFX m_leftFront = new TalonFX(DiffConstants.IDs.FRONT_LEFT_MOTOR.id);
        TalonFX m_rightFront = new TalonFX(DiffConstants.IDs.FRONT_RIGHT_MOTOR.id);
        TalonFX m_leftRear = new TalonFX(DiffConstants.IDs.REAR_LEFT_MOTOR.id);
        TalonFX m_rightRear = new TalonFX(DiffConstants.IDs.REAR_RIGHT_MOTOR.id);
        PneumaticsControlModule m_Hub = new PneumaticsControlModule();

        map.m_DiffDrive = new DiffReal(
            m_leftFront, m_rightFront,
            m_leftRear, m_rightRear,
            m_Hub
        );

        // Pigeon2 pigeon2 = new Pigeon2(10);
        // map.m_gyro = new GyroPigeon2(pigeon2);%

        return map;
    }

    public static RobotMap configureSim() {
        RobotMap map = new RobotMap();

        DiffSim sim = new DiffSim();
        map.m_DiffDrive = sim;
        // map.m_gyro = new GyroSim(sim.getSimAngle());

        return map;
    }
    
}