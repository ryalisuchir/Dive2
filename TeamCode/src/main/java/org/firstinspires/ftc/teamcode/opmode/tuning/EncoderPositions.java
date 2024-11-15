package org.firstinspires.ftc.teamcode.opmode.tuning;


import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Photon
@Autonomous
public class EncoderPositions extends OpMode {

    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);

        telemetry.addData("Ready: ", "Initialized robot.");
        telemetry.update();

    }

    @Override
    public void loop() {
        //Motors:
        telemetry.addLine("Motor Encoders:");
        telemetry.addData("leftLift: ", robot.leftLift.getCurrentPosition());
        telemetry.addData("rightLift: ", robot.rightLift.getCurrentPosition());
        telemetry.addData("extendoMotor: ", robot.extendoMotor.getCurrentPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Closed OpMode.");
    }
}