package org.firstinspires.ftc.teamcode.Claw.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Claw.Claw;

import static org.firstinspires.ftc.teamcode.Claw.Claw.BASE;
import static org.firstinspires.ftc.teamcode.Claw.Claw.DOWN;
import static org.firstinspires.ftc.teamcode.Claw.Claw.UP;

@Autonomous(group = "Test")
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        while (!isStopRequested()) {
            claw.setPosition(UP);
            sleep(500);
            claw.setPosition(DOWN);
            sleep(500);
            claw.setPosition(BASE);
            sleep(500);
        }
    }
}
