package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Autonomous
public class EncoderValues extends LinearOpMode {
    private RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
       robot.init(hardwareMap, telemetry);
       robot.enabled = true;
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addLine("Drivetrain:"); //Drivetrain
            telemetry.addData("Left Front Position: ", robot.leftFront.getCurrentPosition());
            telemetry.addData("Right Front Position: ", robot.rightFront.getCurrentPosition());
            telemetry.addData("Left Rear Position: ", robot.leftRear.getCurrentPosition());
            telemetry.addData("Right Rear Position: ", robot.rightRear.getCurrentPosition());
            telemetry.addLine("Intake:"); //Intake
            telemetry.addData("Extendo: ", robot.extendoMotor.getCurrentPosition());
            telemetry.addLine("Outtake:"); //Outtake
            telemetry.addData("Left Slide: ", robot.leftLift.getCurrentPosition());
            telemetry.addData("Right Slide: ", robot.rightLift.getCurrentPosition());
            telemetry.update();
        }
    }
}