package frc4388.robot.subsystems.differential;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.status.Queryable;
import frc4388.utility.status.Status;
import frc4388.utility.structs.Drivebase;
import pabeles.concurrency.IntOperatorTask.Max;

public class DiffDrive extends SubsystemBase implements Drivebase, Queryable {
    DiffIO io;
    DiffStateAutoLogged state = new DiffStateAutoLogged();
    GyroIO gyroIO;
    GyroStateAutoLogged gyroState = new GyroStateAutoLogged();


    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DiffConstants.TRACK_WIDTH);
    DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0, 0, new Pose2d());

    public DiffDrive(DiffIO io, GyroIO gyroIO) {
        this.io = io;
        this.gyroIO = gyroIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(state);
        Logger.processInputs(getName(), state);
        gyroIO.updateInputs(gyroState);
        Logger.processInputs("gyro", gyroState);

        var speeds = new DifferentialDriveWheelPositions(
            DiffConstants.WHEEL_RADIUS_TO_ARC.times(-state.leftOutputPosition),
            DiffConstants.WHEEL_RADIUS_TO_ARC.times(-state.rightOutputPosition)
        );

        odometry.update(gyroState.yaw, speeds);
    }

    @Override
    public void arcadeDrive(Translation2d left, Translation2d right) {
        io.driveWithInput(
            left.getY() + right.getX(), 
            left.getY() - right.getX()
        );
    }

    private PID rotPid = new PID(DiffConstants.ROT_GAINS); 

    @Override
    public void driveFieldRelative(Translation2d left, Translation2d right) {
        double magnatude = right.getNorm();
        // In case the driver's stick is inside the deadband
        if(magnatude == 0) {
            io.driveWithInput(0, 0);
            return;
        };

        double targetAngle = right.getAngle().getRotations() + 0.25;
        double curAngle = gyroState.yaw.getRotations();

        double error = targetAngle - signedFloor(curAngle);
        if(error > 0.5) {
            error -= 1;
        }else if(error < -0.5) {
            error += 1;
        }
        SmartDashboard.putNumber("error", error);

        double move = (0.5 - Math.abs(error))*2 * magnatude;
        double rot = Math.max(rotPid.update(error) * magnatude, magnatude);

        io.driveWithInput(
            move + rot, 
            move - rot
        );
    }

    private double signedFloor(double x){
        if(x > 0) {
            return x - Math.floor(x);
        } else {
            return Math.ceil(x) - x;
        }
    }

    @Override
    public void resetOdometry() {
        gyroIO.reset();
        odometry.resetPose(new Pose2d());
    }



    @AutoLogOutput
    public Pose2d estimatedPose() {
        return odometry.getEstimatedPosition();
    }


    @Override
    public String getName() {
        return "Diff drive";
    }

    @Override
    public Status diagnosticStatus() {
        return new Status();
    }

}
