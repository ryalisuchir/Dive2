package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;

public class DriveSubsystem extends SubsystemBase {
    private final PinpointDrive drive;
    private final boolean fieldCentric;

    public DriveSubsystem(PinpointDrive drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(drive.pose.position, drive.pose.heading);
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void setPoseEstimate(Pose2d initialPose) {
        drive.pose = initialPose;
    }

    public void setDrivePowers(double x, double y, double angle) {
        drive.setDrivePowers((new PoseVelocity2d(
                        new Vector2d(
                                x,
                                y
                        ),
                        angle
                ))
        );
    }

    public TrajectoryActionBuilder trajectoryActionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }


}
