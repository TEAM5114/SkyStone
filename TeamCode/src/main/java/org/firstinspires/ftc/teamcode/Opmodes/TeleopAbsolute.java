/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Robot.HardwareSkyStone;

/**
 * This teleop opmode controls the robot in absolute coordinates (movement is
 * relative to the driver station and independent of the robot's orientation)
 */

@TeleOp(name="Mecanum absolute", group="Comp")
//@Disabled
public class TeleopAbsolute extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSkyStone robot           = new HardwareSkyStone();
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        double x;
        double y;
        double rotation;
        double magnitude;
        double heading;
        double orientation;
        double angle;
        double v_left_front;
        double v_left_rear;
        double v_right_front;
        double v_right_rear;
        double scalepower;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.update();

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rotation = gamepad1.right_stick_x;
            magnitude = Math.hypot(x, y);
            heading = Math.atan2(y, x);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            orientation = angles.firstAngle;
            angle = heading - orientation;


            v_left_front = magnitude * Math.cos(angle - Math.PI/4) + rotation;
            v_left_rear = magnitude * Math.sin(angle - Math.PI/4) + rotation;
            v_right_front = magnitude * Math.sin(angle - Math.PI/4) - rotation;
            v_right_rear = magnitude * Math.cos(angle - Math.PI/4) - rotation;

            scalepower=Math.max(Math.max(Math.abs(v_left_front),Math.abs(v_left_rear)),Math.max(Math.abs(v_right_front),Math.abs(v_right_rear)));
            if(scalepower>1) {
                v_left_front = v_right_front/scalepower;
                v_left_rear =  v_left_rear/scalepower;
                v_right_front= v_right_front/scalepower;
                v_right_rear=v_right_rear/scalepower;

            }


            robot.leftFrontDrive.setPower(v_left_front);
            robot.leftRearDrive.setPower(v_left_rear);
            robot.rightFrontDrive.setPower(v_right_front);
            robot.rightRearDrive.setPower(v_right_rear);
        }
    }
}
