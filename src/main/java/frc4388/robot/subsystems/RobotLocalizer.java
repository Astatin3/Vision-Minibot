package frc4388.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.RobotGyro;

public class RobotLocalizer extends SubsystemBase {
    private RobotGyro gyro;
    private Pose2d lastPose2d = new Pose2d();
    private PhotonCamera camera;


    public RobotLocalizer(RobotGyro gyro, PhotonCamera cam) {
        this.gyro = gyro;
        this.camera = cam;
    }

    @Override
    public void periodic() {
        // time
        Translation3d accel = gyro.getAcceleration3d();

        SmartDashboard.putNumber("Accel X", accel.getX());
        SmartDashboard.putNumber("Accel Y", accel.getY());
        SmartDashboard.putNumber("Accel Z", accel.getZ());

        Rotation3d rot = gyro.getRotation3d();

        SmartDashboard.putNumber("Rot X", rot.getX());
        SmartDashboard.putNumber("Rot Y", rot.getY());
        SmartDashboard.putNumber("Rot Z", rot.getZ());

        // boolean tagExists = SmartDashboard.getBoolean("photonvision/Camera_Module_v1/hasTarget", false);
        
        Translation2d pos;

        var result = camera.getAllUnreadResults();
        // if (result.hasTargets()) {
        //     PhotonTrackedTarget target = result.getBestTarget();
        //     Transform3d pos3d = target.getBestCameraToTarget();
        //     pos = new Translation2d(pos3d.getX(), pos3d.getY());
        // } else {
            pos = lastPose2d.getTranslation();
        // }

        // results.
        // if (!results.isEmpty()) {

        //     double totalX = 0;
        //     double totalY = 0;

        //     // Camera processed a new frame since last
        //     // Get the last one in the list.
        //     var result = results.get(results.size() - 1);
        //     // PhotonTrackedTarget targets = result.getMultiTagResult();

        //     if (result.hasTargets()) {
        //         PhotonTrackedTarget target = result.getBestTarget();
        //         Transform3d pos3d = target.getBestCameraToTarget();
        //         pos = new Translation2d(pos3d.getX(), pos3d.getY());
        //     } else {
        //         pos = lastPose2d.getTranslation();
        //     }
        // }else {
        //     pos = lastPose2d.getTranslation();
        // }


        lastPose2d = new Pose2d(
            pos,
            gyro.getRotation2d() 
        );
    }

    // Pathplanner function
    public Pose2d getPose(){
        return lastPose2d;
    }

    // PathPlanner func
    public void resetPose(Pose2d pose){
        lastPose2d = pose;
    }

    // PathPlanner
    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
            0, 
            0, 
            Units.rotationsToRadians(gyro.getAngularVelocity())
        );
    }
}
