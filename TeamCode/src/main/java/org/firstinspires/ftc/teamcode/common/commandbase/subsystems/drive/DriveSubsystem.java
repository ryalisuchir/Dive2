package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.roadrunner.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final boolean fieldCentric;

    public DriveSubsystem(MecanumDrive drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public Pose2d getPoseEstimate() {
        return drive.localizer.getPose();
    }

    public void setPoseEstimate(Pose2d initialPose) {
        drive.localizer.setPose(initialPose);
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public TrajectoryActionBuilder trajectoryActionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }


}
