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

    private RobotHardware robot = new RobotHardware();
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        robot.init(hardwareMap, Globals.DEFAULT_START_POSE);

        telemetry.addData("Ready: ", "Initialized robot.");
        telemetry.update();

    }

    @Override
    public void loop() {
        //Motors:
        telemetry.addLine("Motor Encoders:");
        telemetry.addData("leftFront: ", robot.leftFront.getCurrentPosition());
        telemetry.addData("rightFront: ", robot.rightFront.getCurrentPosition());
        telemetry.addData("leftRear: ", robot.leftRear.getCurrentPosition());
        telemetry.addData("rightRear: ", robot.rightRear.getCurrentPosition());
//        telemetry.addData("leftLift: ", robot.leftLift.getCurrentPosition());
//        telemetry.addData("rightLift: ", robot.rightLift.getCurrentPosition());
//        telemetry.addData("extendoMotor: ", robot.extendoMotor.getCurrentPosition());

        //Servos:
        telemetry.addLine("Servos Encoders:");
        telemetry.addData("intakeRotation: ", robot.intakeRotation.getPosition());
        telemetry.addData("intakeClaw: ", robot.intakeClaw.getPosition());
        telemetry.addData("intakeCoaxial1: ", robot.intakeCoaxial1.getPosition());
//        telemetry.addData("intakeCoaxial1: ", robot.intakeCoaxial2.getPosition());
        telemetry.addData("intake4Bar2: ", robot.intake4Bar1.getPosition());
        telemetry.addData("intake4Bar2: ", robot.intake4Bar2.getPosition());
//        telemetry.addData("outtakeRotation: ", robot.outtakeRotation.getPosition());
//        telemetry.addData("leftOuttakeArm: ", robot.leftOuttakeArm.getPosition());
//        telemetry.addData("rightOuttakeArm: ", robot.rightOuttakeArm.getPosition());
//        telemetry.addData("outtakeClaw: ", robot.outtakeClaw.getPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Closed OpMode.");
    }
}