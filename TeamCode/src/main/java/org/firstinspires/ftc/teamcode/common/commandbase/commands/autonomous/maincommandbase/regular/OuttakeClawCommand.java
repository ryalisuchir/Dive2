package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.regular;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class OuttakeClawCommand extends CommandBase {
    OuttakeClawSubsystem outtakeClawSubsystem;
    Globals.OuttakeClawState outtakeClawState;
    double setPosition;

    public OuttakeClawCommand(OuttakeClawSubsystem outtakeClawSubsystemInput, Globals.OuttakeClawState outtakeClawStateInput) {
        this.outtakeClawSubsystem = outtakeClawSubsystemInput;
        this.outtakeClawState = outtakeClawStateInput;

        addRequirements(outtakeClawSubsystem);
    }

    @Override
    public void initialize(){
        switch (outtakeClawState) {
            case OPEN:
                setPosition = Globals.OUTTAKE_CLAW_OPEN;
                outtakeClawSubsystem.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                setPosition = Globals.OUTTAKE_CLAW_CLOSED;
                outtakeClawSubsystem.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_CLOSED);
                break;
            case OPEN_TRANSFER:
                setPosition = Globals.OUTTAKE_CLAW_TRANSFER;
                outtakeClawSubsystem.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_TRANSFER);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(setPosition - outtakeClawSubsystem.outtakeClaw.getPosition()) < 0.0001);
    }

}
