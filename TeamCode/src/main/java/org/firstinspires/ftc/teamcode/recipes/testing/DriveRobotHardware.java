package org.firstinspires.ftc.teamcode.recipes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;

@Config
public class DriveRobotHardware {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear; //Drivetrain motors

    public DriveSubsystem driveSubsystem;

    private double voltage = 0.0;
    private ElapsedTime voltageTimer;

    private static DriveRobotHardware instance = null;
    public boolean enabled;
    private HardwareMap hardwareMap;
    GoBildaPinpointDriverRR pinPointDriver;

    public static DriveRobotHardware getInstance() {
        if (instance == null) {
            instance = new DriveRobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Pose2d initialPose) {
        voltageTimer = new ElapsedTime();
        //Configuration of all motors:
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //Reversing motors:
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        //Setting all motors to stop:
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveSubsystem = new DriveSubsystem(new PinpointDrive(hardwareMap, initialPose), false);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        CommandScheduler.getInstance().registerSubsystem(driveSubsystem);
    }

    public double getVoltage() {
        return voltage;
    }
}