package frc4388.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants.AutoConstants;

public class TankDrive extends SubsystemBase{
    private TalonSRX FR;
    private TalonSRX FL;
    private TalonSRX BL;
    private TalonSRX BR;
    private RobotLocalizer robotLocalizer;

    public TankDrive(TalonSRX FR,TalonSRX FL,TalonSRX BL, TalonSRX BR, RobotLocalizer robotLocalizer){
        this.FR = FR;
        this.FL = FL;
        this.BL = BL;
        this.BR = BR;
        this.robotLocalizer = robotLocalizer;

        
        // Configure AutoBuilder last
        AutoBuilder.configure(
            robotLocalizer::getPose,
            robotLocalizer::resetPose,
            robotLocalizer::getChassisSpeeds,
            this::setTargetChassisSpeeds,
            AutoConstants.PP_PATH_FOLLOWING_CONTROLLER,
            AutoConstants.PP_ROBOT_CONFIG,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    private static final ControlMode mode = ControlMode.PercentOutput;

    private static final double maxMoveSpeed = 0.3;
    private static final double maxturnSpeed = 0.3;


    private static final double[] turn = { //Right by default, motors are wired weirdly
        maxturnSpeed, //FR
        maxturnSpeed, //FL
        maxturnSpeed, //BL
        maxturnSpeed, //BR
    };

    private static final double[] move = { //forward by default, motors are wired weirdly
        -maxMoveSpeed,  //FR
        maxMoveSpeed,  //FL
        maxMoveSpeed,  //BL
        -maxMoveSpeed,  //BR
    };

    public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {

        double percent_move = /* (1. - Math.abs(rightStick.getX())) */ leftStick.getY();

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

    private double clamp(double x, double min, double max){
        return Math.max(Math.min(x, max), min);
    }

    private void setTargetChassisSpeeds(ChassisSpeeds target, DriveFeedforwards idk) {
        double move = clamp(target.vxMetersPerSecond/AutoConstants.MAX_DRIVE_VELOCITY, -1, 1);
        double rot = clamp(Units.radiansToRotations(target.omegaRadiansPerSecond)/AutoConstants.MAX_ROT_SPEED, -1, 1);

        driveWithInput(
            new Translation2d(0, move), 
            new Translation2d(rot, 0),
            false);

    }
}
