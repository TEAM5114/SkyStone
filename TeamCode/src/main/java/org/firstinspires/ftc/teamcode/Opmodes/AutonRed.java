package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.security.Policy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

@Autonomous(group = "Comp")
public class AutonRed extends OpMode {
    //TODO: Enter correct coordinates!!!
    MecanumBase drive;
    Claw claw;
    State state;
    List<Recognition> recognitions = new ArrayList<>();
    TreeMap<Float, String> treeMap = new TreeMap<>(Collections.reverseOrder());
    NanoClock clock = NanoClock.system();
    Vector2d firstStoneLocation, secondStoneLocation;
    boolean secondStoneCaptured = false;

    enum State {
        DETECT_STONE,
        CAPTURE_FIRST_STONE,
        DELIVER_STONE,
        CAPTURE_SECOND_STONE,
        MOVE_FOUNDATION,
        PARK;
    }

    @Override
    public void init() {
        drive = new MecanumREV(hardwareMap);
        claw = new Claw(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-35, -65, -Math.PI/2));
        state = State.DETECT_STONE;
        claw.setPositionSync(0.5);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop() {
        switch (state){
            case DETECT_STONE:
                Trajectory trajectory = drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(-35, -55))
                        .build();
                drive.followTrajectorySync(trajectory);
                int skystone = drive.detector.detectSkystone(500);
                switch (skystone){
                    case 1:
                        firstStoneLocation = new Vector2d(-35, -35);
                        break;
                    case 2:
                        firstStoneLocation = new Vector2d(-35, -43);
                        break;
                    default:
                        firstStoneLocation = new Vector2d(-35, -51);
                }
                secondStoneLocation = firstStoneLocation.plus(new Vector2d(0, -24));
                state = State.CAPTURE_FIRST_STONE;
                break;
            case CAPTURE_FIRST_STONE:
                trajectory = drive.trajectoryBuilder()
                        .strafeTo(firstStoneLocation).build();
                drive.followTrajectorySync(trajectory);
                claw.setRightClawPositionSync(claw.DOWN);
                state = State.DELIVER_STONE;
                break;
            case DELIVER_STONE:
                Pose2d currentPose = drive.getPoseEstimate();
                Vector2d currentLocation = new Vector2d(currentPose.getX(), currentPose.getY());
                trajectory = drive.trajectoryBuilder()
                        .lineTo(currentLocation.plus(new Vector2d(-20, 0)),
                                new LinearInterpolator(-Math.PI/2, -Math.PI/2))
                        .strafeTo(new Vector2d(20, drive.getPoseEstimate().getY()))
                        .build();
                drive.followTrajectorySync(trajectory);
                claw.setRightClawPositionSync(claw.UP);
                if (secondStoneCaptured){
                    state = State.MOVE_FOUNDATION;
                } else {
                    state = State.CAPTURE_SECOND_STONE;
                }
                break;
            case CAPTURE_SECOND_STONE:
                trajectory = drive.trajectoryBuilder()
                        .lineTo(firstStoneLocation.plus(new Vector2d(-20, 0)))
                        .lineTo(secondStoneLocation.plus(new Vector2d(-20, 0)),
                                new LinearInterpolator(0, Math.PI/2))
                        .strafeTo(secondStoneLocation).build();
                drive.followTrajectorySync(trajectory);
                claw.setRightClawPositionSync(claw.DOWN);
                secondStoneCaptured = true;
                state = State.DELIVER_STONE;
                break;
            case MOVE_FOUNDATION:
                // lineTo something rotate -PI/2
                //strafeTo something
                //strafeTO foundation
                //grab foundatione
                //strafeTo Wall
                //release foundation
                state = State.PARK;
                break;
            case PARK:
                trajectory = drive.trajectoryBuilder()
                        .lineTo(new Vector2d(0, drive.getPoseEstimate().getY()),
                                new LinearInterpolator(0, -Math.PI/2))
                        .strafeTo(new Vector2d(0, -20))
                        .build();
                drive.followTrajectorySync(trajectory);
                break;
        }
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
