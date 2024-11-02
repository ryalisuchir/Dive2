package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TransfersOuttakes extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
