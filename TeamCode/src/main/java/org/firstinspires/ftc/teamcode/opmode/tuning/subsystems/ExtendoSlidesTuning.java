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
public class ExtendoSlidesTuning extends LinearOpMode {

    public static double p = 0.011;
    public static double i = 0;
    public static double d = 0.0002;
    public static double f = 0.00016;
    private static final PIDFController extendoPIDF = new PIDFController(p, i, d, f);
    public static double setPoint = 0;
    public static double maxPowerConstant = 1.0;
    public DcMotorEx extendoMotor;
    int motorPos = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");

        extendoMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Set motors to use encoder mode
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Variables for PD control
        waitForStart();

        while (opModeIsActive()) {
            // Calculate error
            motorPos = extendoMotor.getCurrentPosition();

            extendoPIDF.setP(p);
            extendoPIDF.setI(i);
            extendoPIDF.setD(d);
            extendoPIDF.setF(f);

            extendoPIDF.setSetPoint(setPoint);

            double maxPower = (f * motorPos) + maxPowerConstant;
            double power = Range.clip(extendoPIDF.calculate(motorPos, setPoint), -maxPower, maxPower);

            extendoMotor.setPower(power);

            // Send telemetry data to FTC Dashboard
            telemetry.addData("Target Position", setPoint);
            telemetry.addData("Current Position", extendoMotor.getCurrentPosition());
            telemetry.addData("Error", setPoint - extendoMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
