package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.io.FileWriter;

public class Auton extends OpMode {
    //TODO: Enter correct coordinates!!!
    MecanumBase drive;
    Claw claw;
    State state;
    Vector2d firstStoneCoordinates, secondStoneCoordinates, detectCoordinates, parkCoordinates;
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
        detectCoordinates = drive.getPoseEstimate().vec().plus(new Vector2d(0, 7));
        stonesLocation = new Vector2d(0, -33);
        stonesToDeliveryLocation = new Vector2d(0, -1).times(10);
        parkCoordinates = new Vector2d(0, -34);

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
                        .lineTo(detectCoordinates)
                        .build();
                drive.followTrajectorySync(trajectory);

                int skystone = drive.detector.detectSkystone(500);
                switch (skystone){
                    case 1:
                        firstStoneCoordinates = stonesLocation.plus(new Vector2d(-24, 0));
                        break;
                    case 2:
                        firstStoneCoordinates = stonesLocation.plus(new Vector2d(-32, 0));
                        break;
                    default:
                        firstStoneCoordinates = stonesLocation.plus(new Vector2d(-40, 0));
                }

                stageDeliveryLocation = firstStoneCoordinates.plus(stonesToDeliveryLocation);
                deliveryLocation = new Vector2d(30, stageDeliveryLocation.getY());
                secondStoneCoordinates = firstStoneCoordinates.plus(new Vector2d(-24, 0));

                state = State.CAPTURE_FIRST_STONE;

                break;

            case CAPTURE_FIRST_STONE:
                trajectory = drive.trajectoryBuilder()
                        .strafeTo(firstStoneCoordinates).build();
                drive.followTrajectorySync(trajectory);

                claw.setRightClawPositionSync(claw.DOWN);

                state = State.DELIVER_STONE;

                break;

            case DELIVER_STONE:
                Vector2d loc;
                if (secondStoneCaptured) {
                    loc = stageDeliveryLocation.plus(new Vector2d(-24, 0));
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
                        .strafeTo(stageDeliveryLocation)
                        .reverse()
                        .lineTo(stageDeliveryLocation.plus(new Vector2d(-24, 0)),
                                new LinearInterpolator(deliverHeading,
                                        startHeading - deliverHeading))
                        .strafeTo(secondStoneCoordinates).build();
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
                        .strafeTo(parkCoordinates)
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
