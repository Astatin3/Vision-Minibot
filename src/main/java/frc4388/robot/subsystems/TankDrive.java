package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase{
    private TalonSRX FR;
    private TalonSRX FL;
    private TalonSRX BL;
    private TalonSRX BR;

    public TankDrive(TalonSRX FR,TalonSRX FL,TalonSRX BL, TalonSRX BR){
        this.FR = FR;
        this.FL = FL;
        this.BL = BL;
        this.BR = BR;
    }

    private static final ControlMode mode = ControlMode.PercentOutput;

    private static final double maxMoveSpeed = 0.6;
    private static final double maxturnSpeed = 0.6;


    private static final double[] turn = { //Right by default, motors are wired weirdly
        maxturnSpeed, //FR
        maxturnSpeed,  //FL
        maxturnSpeed,  //BL
        maxturnSpeed, //BR
    };

    private static final double[] move = { //forward by default, motors are wired weirdly
        -maxMoveSpeed,  //FR
        maxMoveSpeed,  //FL
        maxMoveSpeed,  //BL
        -maxMoveSpeed,  //BR
    };

    public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {

        double percent_move = (1. - Math.abs(rightStick.getX())) * leftStick.getY();

        double FR_rot = move[0] * percent_move + turn[0] * rightStick.getX();
        double FL_rot = move[1] * percent_move + turn[1] * rightStick.getX();
        double BL_rot = move[2] * percent_move + turn[2] * rightStick.getX();
        double BR_rot = move[3] * percent_move + turn[3] * rightStick.getX();

        FR.set(mode, FR_rot);
        FL.set(mode, FL_rot);
        BL.set(mode, BL_rot);
        BR.set(mode, BR_rot);

        SmartDashboard.putNumber("FR", FR_rot);
        SmartDashboard.putNumber("FL", FL_rot);
        SmartDashboard.putNumber("BL", BL_rot);
        SmartDashboard.putNumber("BR", BR_rot);
    }
}
