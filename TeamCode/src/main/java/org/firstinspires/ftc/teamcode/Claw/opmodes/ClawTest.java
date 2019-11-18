package org.firstinspires.ftc.teamcode.Claw.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Claw.Claw;

import static org.firstinspires.ftc.teamcode.Claw.Claw.BASE;
import static org.firstinspires.ftc.teamcode.Claw.Claw.DOWN;
import static org.firstinspires.ftc.teamcode.Claw.Claw.UP;

@TeleOp(group = "Test")
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                claw.setLeftClawPosition(DOWN);
            }
            if (gamepad1.dpad_right){
                claw.setRightClawPosition(DOWN);
            }
            if (gamepad1.dpad_down){
                claw.setPosition(DOWN);
            }
            if (gamepad1.dpad_up){
                claw.setPosition(UP);
            }
        }
    }
}
