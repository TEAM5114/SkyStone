package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="IMU odometry test",group = "Test")
public class IMUOdometryTest extends LinearOpMode {

    Robot robot = new Robot();
    Position position;

    @Override
    public void runOpMode(){
        double x;
        double y;
        double pivot;
        double magnitude;
        double heading;
        boolean DRIVER_RELATIVE = true; //for robot-relative control, set to false


        robot.init(hardwareMap);

        composeTelemetry();

        waitForStart();
        robot.startIMUAccelerationIntegration();

        while (opModeIsActive()) {

            telemetry.update();

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;

            magnitude = Math.hypot(x, y);
            heading = Math.atan2(y, x);

            robot.run(magnitude, heading, pivot, DRIVER_RELATIVE);
        }
    }

    void composeTelemetry() {

          telemetry.addAction(new Runnable() {
              @Override public void run(){
                 position = robot.getPosition();
              }
          });

         telemetry.addLine()
                .addData("position", new Func<String>() {
                    @Override public String value() {
                        return position.toString();
                    }
                });
    }
}
