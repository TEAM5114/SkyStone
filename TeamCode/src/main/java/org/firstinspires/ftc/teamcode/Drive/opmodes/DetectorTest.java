package org.firstinspires.ftc.teamcode.Drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

@TeleOp(group = "Test")
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumBase drive = new MecanumREV(hardwareMap);
        Pose2d stoneLocation;

        drive.setPoseEstimate(new Pose2d(-40, -60, Math.toRadians(90)));
        waitForStart();

        while (!isStopRequested()){
            drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            if (drive.detector.skystoneVisible()) {
                telemetry.addLine("Skystone Visible");
            }

            telemetry.update();
        }
    }
}
