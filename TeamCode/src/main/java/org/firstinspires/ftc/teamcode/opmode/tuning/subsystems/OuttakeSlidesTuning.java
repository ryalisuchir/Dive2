package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Slides Test with Dashboard", group = "Test")
public class OuttakeSlidesTuning extends LinearOpMode {

    public static double p = 0.011;
    public static double i = 0;
    public static double d = 0.0002;
    public static double f = 0.00016;
    private static final PIDFController extendoPIDF = new PIDFController(p, i, d, f);
    public static double setPoint = 0;
    public static double maxPowerConstant = 1.0;
    public DcMotorEx leftLift, rightLift;
    int motorPos = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        rightLift.setDirection(DcMotorEx.Direction.REVERSE);

        // Set motors to use encoder mode
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Variables for PD control
        waitForStart();

        while (opModeIsActive()) {
            // Calculate error
            motorPos = rightLift.getCurrentPosition();

            extendoPIDF.setP(p);
            extendoPIDF.setI(i);
            extendoPIDF.setD(d);
            extendoPIDF.setF(f);

            extendoPIDF.setSetPoint(setPoint);

            double maxPower = (f * motorPos) + maxPowerConstant;
            double power = Range.clip(extendoPIDF.calculate(motorPos, setPoint), -maxPower, maxPower);

            rightLift.setPower(power);
            leftLift.setPower(power);

            // Send telemetry data to FTC Dashboard
            telemetry.addData("Target Position", setPoint);
            telemetry.addData("Current Position", rightLift.getCurrentPosition());
            telemetry.addData("Error", setPoint - rightLift.getCurrentPosition());
            telemetry.update();
        }
    }
}
