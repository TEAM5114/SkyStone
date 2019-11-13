package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Teleop",group = "Test")
public class Teleop extends LinearOpMode {

    boolean ROBOT_RELATIVE = true; //for robot-relative control, set to false

    @Override
    public void runOpMode(){
        MecanumBase drive = new MecanumREV(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-40, -60, Math.toRadians(90)));

        waitForStart();

        while (!isStopRequested()){
            if (gamepad1.a) {
                ROBOT_RELATIVE = !ROBOT_RELATIVE;
                claw.setPosition(ROBOT_RELATIVE ? 0 : 0.5);
            }
            drive.setDrivePowerRel(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x),
                    ROBOT_RELATIVE);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.log().clear();
            telemetry.log().add("%s relative steering", ROBOT_RELATIVE?"Robot":"Driver");
            telemetry.log().add(Misc.formatInvariant("x %0.2d" , poseEstimate.getX()));
            telemetry.log().add(Misc.formatInvariant("y %0.2d" , poseEstimate.getY()));
            telemetry.log().add(Misc.formatInvariant("heading %0.2d" , poseEstimate.getHeading()));
            telemetry.update();
        }
    }
}
