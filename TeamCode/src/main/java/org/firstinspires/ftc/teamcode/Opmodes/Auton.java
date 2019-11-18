package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.TreeMap;

public class Auton extends OpMode {
    //TODO: Enter correct coordinates!!!
    MecanumBase drive;
    Claw claw;
    State state;
    Vector2d firstStoneLocation, secondStoneLocation, detectLocation, parkLocation;
    Vector2d stonesLocation, stonesToDeliveryLocation, stageDeliveryLocation, deliveryLocation;
    Trajectory trajectory;
    double startHeading, deliverHeading;
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
        state = State.DETECT_STONE;
        claw.setPositionSync(0.5);
        drive.setPoseEstimate(new Pose2d(-33, -66, -Math.PI/2));
        startHeading = drive.getPoseEstimate().getHeading();
        deliverHeading = -Math.PI;
        detectLocation = drive.getPoseEstimate().vec().plus(new Vector2d(0, 10));
        stonesLocation = new Vector2d(0, -33);
        stonesToDeliveryLocation = new Vector2d(0, -1).times(10);
        parkLocation = new Vector2d(0, -34);

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop() {
        switch (state){
            case DETECT_STONE:
                trajectory = drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(detectLocation)
                        .build();
                drive.followTrajectorySync(trajectory);

                int skystone = drive.detector.detectSkystone(500);
                switch (skystone){
                    case 1:
                        firstStoneLocation = stonesLocation.plus(new Vector2d(-24, 0));
                        break;
                    case 2:
                        firstStoneLocation = stonesLocation.plus(new Vector2d(-32, 0));
                        break;
                    default:
                        firstStoneLocation = stonesLocation.plus(new Vector2d(-40, 0));
                }

                stageDeliveryLocation = firstStoneLocation.plus(stonesToDeliveryLocation);
                deliveryLocation = new Vector2d(30, stageDeliveryLocation.getY());
                secondStoneLocation = firstStoneLocation.plus(new Vector2d(-24, 0));

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
                Vector2d loc;
                if (secondStoneCaptured) {
                    loc = stageDeliveryLocation.plus(new Vector2d(0, 24));
                    state = State.MOVE_FOUNDATION;
                } else {
                    loc = stageDeliveryLocation;
                    state = State.CAPTURE_SECOND_STONE;
                }

                trajectory = drive.trajectoryBuilder()
                        .lineTo(loc,
                                new LinearInterpolator(startHeading,
                                        deliverHeading - startHeading))
                        .strafeTo(deliveryLocation)
                        .build();
                drive.followTrajectorySync(trajectory);

                claw.setRightClawPositionSync(claw.UP);

                break;

            case CAPTURE_SECOND_STONE:
                trajectory = drive.trajectoryBuilder()
                        .lineTo(stageDeliveryLocation)
                        .lineTo(stageDeliveryLocation.plus(new Vector2d(0, 24)),
                                new LinearInterpolator(deliverHeading,
                                        startHeading - deliverHeading))
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
                                new LinearInterpolator(deliverHeading,
                                        startHeading - deliverHeading))
                        .strafeTo(parkLocation)
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
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
