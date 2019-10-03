package org.firstinspires.ftc.teamcode.Opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Color sensor", group = "Test")
public class ColorSensorTest extends LinearOpMode {
    float[] hsvValues = new float[3];
    Robot robot = new Robot();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            NormalizedRGBA colors = robot.getColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            telemetry.update();
        }
    }

}
