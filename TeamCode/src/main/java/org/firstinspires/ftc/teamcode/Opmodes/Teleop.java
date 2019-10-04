package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Teleop",group = "Test")
public class Teleop extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode(){
        double x;
        double y;
        double pivot;
        double magnitude;
        double heading;
        boolean DRIVER_RELATIVE = true; //for robot-relative control, set to false

        robot.init(hardwareMap);

        waitForStart();

        robot.startIMUAccelerationIntegration();

        telemetry.addLine("Driver-station relative steering.");
        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                DRIVER_RELATIVE = !DRIVER_RELATIVE;
            }
            if (DRIVER_RELATIVE){
                telemetry.addLine("Driver-station relative steering.");
            } else {
                telemetry.addLine("Robot relative steering.");
            }

            telemetry.update();

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;

            magnitude = Math.hypot(x, y);
            heading = Math.atan2(y, x);

            robot.run(magnitude, heading, pivot, DRIVER_RELATIVE);
        }
    }
}
