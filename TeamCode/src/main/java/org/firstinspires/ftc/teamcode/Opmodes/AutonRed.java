package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;

@Autonomous(group = "Test")
public class AutonRed extends OpMode {
    MecanumBase drive = new MecanumREV(hardwareMap);
    Claw claw = new Claw(hardwareMap);

    @Override
    public void init() {
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        try {
            FileWriter fileWriter = new FileWriter("/sdcard/FIRST/heading.txt");
            fileWriter.write(String.format("%.3f", drive.getExternalHeading()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
