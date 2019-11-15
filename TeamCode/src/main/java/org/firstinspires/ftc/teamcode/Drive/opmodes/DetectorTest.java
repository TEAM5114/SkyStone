package org.firstinspires.ftc.teamcode.Drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

@TeleOp(group = "Test")
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumBase drive = new MecanumREV(hardwareMap);
        NanoClock clock = NanoClock.system();
        drive.setPoseEstimate(new Pose2d(-40, -60, Math.toRadians(90)));
        List<Recognition> recognitions = new ArrayList<>();
        TreeMap<Float, String> tm = new TreeMap<>(Collections.reverseOrder());

        waitForStart();

        double loopStart = clock.seconds();
        while (opModeIsActive()) {
            double elapsedTime = clock.seconds() - loopStart;
            recognitions = drive.detector.detectSkystone();
            if (recognitions != null) {
                //            telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : recognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left (%d)", i), "%.03f",
                            recognition.getLeft());
                    i++;
                }
                telemetry.update();
            }
            if (recognitions.size() > 1 || elapsedTime > 500) { break; }
        }
        while (opModeIsActive()){
            if (recognitions == null) {
                telemetry.addLine("No stones detected");
            } else if (recognitions.size() == 1){
                telemetry.addLine("One stone detected");
                telemetry.addData("Stone is ", recognitions.get(0).getLabel());
            } else if (recognitions.size() == 2 || recognitions.size() == 3) {
                for (Recognition recognition : recognitions) {
                    tm.put(recognition.getLeft(), recognition.getLabel());
                }
                int i = 1;
                for (Map.Entry<Float, String> entry : tm.entrySet()){
                    if (entry.getValue().equals("Skystone")) { break; }
                    i++;
                }
                telemetry.addLine(String.format("%s stones detected",
                        recognitions.size() == 2 ? "Two" : "Three"));
                telemetry.addData("Skystone is number ", "%d", i);
            } else {
                telemetry.addLine("More than three stones detected???");
            }
            telemetry.update();
        }

    }
}

