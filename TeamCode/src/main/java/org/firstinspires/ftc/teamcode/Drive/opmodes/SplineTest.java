package org.firstinspires.ftc.teamcode.Drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

@Autonomous(group = "Test")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumBase drive = new MecanumREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder().splineTo(new Pose2d(30, 30, 0)).build()
        );

        sleep(2000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder().splineTo(new Pose2d(0, 0, 0)).build()
        );
    }
}
