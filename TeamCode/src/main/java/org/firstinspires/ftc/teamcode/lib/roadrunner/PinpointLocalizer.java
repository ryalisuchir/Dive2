package org.firstinspires.ftc.teamcode.lib.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;
    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);
    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        driver.setEncoderResolution(19.89436789f);
        driver.setOffsets(DistanceUnit.MM.fromInches(2.486136954703216), DistanceUnit.MM.fromInches(5.203429930760357));

        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX() / 25.4, driver.getPosY() / 25.4, driver.getHeading());
            Vector2d worldVelocity = new Vector2d(driver.getVelX() / 25.4, driver.getVelY() / 25.4);
            Vector2d robotVelocity = Rotation2d.fromDouble(-driver.getHeading()).times(worldVelocity);
            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity());
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}