package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;

public class DriveSubsystem extends SubsystemBase {
    private final PinpointDrive drive;
    private final boolean fieldCentric;

    public DriveSubsystem(PinpointDrive drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public Pose2d getPoseEstimate() {return new Pose2d(drive.pose.position, drive.pose.heading); }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public TrajectoryActionBuilder trajectoryActionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }

}
