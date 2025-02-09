package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Slides Test with Dashboard", group = "Test")
public class ExtendoSlidesTuning extends LinearOpMode {

    // PID coefficients adjustable via FTC Dashboard
    public static double kP = 0;
    public static double kD = 0;
    public static double targetPosition = 0;

    private DcMotorEx extendoMotor;

    @Override
    public void runOpMode() {
        // Initialize motors
        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");

        extendoMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Set motors to use encoder mode
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Connect to FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Variables for PD control
        double error;
        double prevError = 0;
        double derivative;
        double output;

        waitForStart();

        while (opModeIsActive()) {
            // Calculate error
            error = targetPosition - extendoMotor.getCurrentPosition();

            // Calculate derivative
            derivative = error - prevError;

            // PD control output
            output = (kP * error) + (kD * derivative);

            // Apply power to motors
            extendoMotor.setPower(output);

            // Update previous error
            prevError = error;

            // Send telemetry data to FTC Dashboard
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", extendoMotor.getCurrentPosition());
            telemetry.addData("Error", error);
            telemetry.addData("P Term", kP * error);
            telemetry.addData("D Term", kD * derivative);
            telemetry.addData("Output", output);
            telemetry.update();
        }
    }
}
