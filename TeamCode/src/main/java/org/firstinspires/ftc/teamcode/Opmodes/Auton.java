package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.io.FileWriter;

import static java.lang.Thread.sleep;

@Autonomous(group = "Comp")
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
    boolean allianceSet = false;
    Alliance alliance;

    enum Alliance {
        RED,
        BLUE;
    }
    enum State {
        DETECT_STONE,
        CAPTURE_FIRST_STONE,
        DELIVER_STONE,
        STAGE_FOR_SECOND_STONE,
        CAPTURE_SECOND_STONE,
        MOVE_FOUNDATION,
        MOVE_TO_LINE,
        PARK,
        STOP;
    }

    @Override
    public void init() {
        msStuckDetectLoop = 7000;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumREV(hardwareMap);
        claw = new Claw(hardwareMap);

        state = State.DETECT_STONE;
        claw.setPosition(0.5);
        deliverHeading = -Math.PI;

        alliance = Alliance.RED;

        telemetry.addLine("Playing as " + alliance.toString() + " alliance.");
        telemetry.update();
    }

    @Override
    public void init_loop(){
        if (!allianceSet) {
            switch (alliance) {
                case RED:
                    startHeading = -Math.PI / 2;
                    drive.setPoseEstimate(new Pose2d(-33, -66, startHeading));
                    detectCoordinates = drive.getPoseEstimate().vec().plus(new Vector2d(0, 5));
                    stonesLocation = new Vector2d(0, -33);
                    stonesToDeliveryLocation = new Vector2d(0, -20);
                    parkCoordinates = new Vector2d(0, -34);
                    break;
                case BLUE:
                    startHeading = Math.PI/2;
                    drive.setPoseEstimate(new Pose2d(-33, 66, startHeading));
                    detectCoordinates = drive.getPoseEstimate().vec().plus(new Vector2d(0, -5));
                    stonesLocation = new Vector2d(0, 33);
                    stonesToDeliveryLocation = new Vector2d(0, 20);
                    parkCoordinates = new Vector2d(0, 34);
                    break;
            }
            allianceSet = true;
        }
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

                int skystone = drive.detector.detectSkystone(1);
                telemetry.addLine("Skystone is " + skystone);
                telemetry.update();
                switch (skystone){
                    case 1:
                        firstStoneCoordinates = stonesLocation.plus(new Vector2d(-22, 0));
                        break;
                    case 2:
                        firstStoneCoordinates = stonesLocation.plus(new Vector2d(-30, 0));
                        break;
                    default:
                        firstStoneCoordinates = stonesLocation.plus(new Vector2d(-38, 0));
                }

                stageDeliveryLocation = firstStoneCoordinates.plus(stonesToDeliveryLocation);
                deliveryLocation = new Vector2d(20, stageDeliveryLocation.getY());
                secondStoneCoordinates = firstStoneCoordinates.plus(new Vector2d(-27, 0));

                state = State.CAPTURE_FIRST_STONE;

                break;

            case CAPTURE_FIRST_STONE:
                trajectory = drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(firstStoneCoordinates, new ConstantInterpolator(drive.getPoseEstimate().getHeading())).build();
                drive.followTrajectorySync(trajectory);

                grabStone();

                state = State.DELIVER_STONE;

                break;

            case DELIVER_STONE:
                Vector2d loc;
                if (secondStoneCaptured) {
                    loc = stageDeliveryLocation;
//                    loc = stageDeliveryLocation.plus(new Vector2d(-24, 0));
                    state = State.MOVE_FOUNDATION;
                } else {
                    loc = stageDeliveryLocation;
                    state = State.STAGE_FOR_SECOND_STONE;
                }

                trajectory = drive.trajectoryBuilder()
                        .lineTo(loc,
                                new LinearInterpolator(startHeading,
                                        deliverHeading - startHeading))
                        .reverse()
                        .lineTo(deliveryLocation)
                        .build();
                drive.followTrajectorySync(trajectory);

                claw.setPosition(0.5);


                break;
            case STAGE_FOR_SECOND_STONE:
                trajectory = drive.trajectoryBuilder()
                        .lineTo(stageDeliveryLocation)
                        .lineTo(stageDeliveryLocation.plus(new Vector2d(-24, 0)),
                                new LinearInterpolator(deliverHeading,
                                        startHeading - deliverHeading)).build();
                drive.followTrajectorySync(trajectory);

                state = State.CAPTURE_SECOND_STONE;

                break;

            case CAPTURE_SECOND_STONE:

                trajectory = drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(secondStoneCoordinates, new ConstantInterpolator(drive.getPoseEstimate().getHeading())).build();
                drive.followTrajectorySync(trajectory);

                grabStone();


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
                                        -1 * (startHeading - deliverHeading)))
//                        .lineTo(parkCoordinates, new ConstantInterpolator(drive.getPoseEstimate().getHeading()))
                        .forward(10)
                        .build();


                drive.followTrajectorySync(trajectory);

                state = State.STOP;
                break;
            case STOP:
                drive.setDrivePower(new Pose2d(0,0, 0));
        }
        telemetry.addLine(drive.getPoseEstimate().toString());
        telemetry.update();
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

    public void waitForServo(double milliSeconds){
        try {
            Thread.sleep(500);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }

    public void grabStone(){
        switch (alliance){
            case RED:
                claw.setRightClawPosition(Claw.CAPTURE);
                break;
            case BLUE:
                claw.setLeftClawPosition(Claw.CAPTURE);
                break;
        }
        waitForServo(500);
    }
}
