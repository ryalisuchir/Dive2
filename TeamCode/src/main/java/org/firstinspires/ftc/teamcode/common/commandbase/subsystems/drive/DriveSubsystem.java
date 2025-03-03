package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.pathing.roadrunner.PinpointDrive;

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

    public void setPoseEstimate(Pose2d initialPose) {
        drive.pose = initialPose;
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public TrajectoryActionBuilder trajectoryActionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }

    public TrajectoryActionBuilder whatTheSigma(Pose2d startPose) {
        return drive.whatTheSigma(startPose);
    }

    public TrajectoryActionBuilder trajectoryActionBuilderWithExtendo(Pose2d startPose) {
        return drive.actionBuilderWithExtendo(startPose);
    }


}
