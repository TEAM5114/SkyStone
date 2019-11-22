package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Detector;
import org.firstinspires.ftc.teamcode.Robot.Drive;

@Autonomous(group = "Test")
public class Auton extends OpMode {
    Drive drive;
//    Claw claw;
    Detector detector;
    State state;
    boolean secondStoneCaptured = false;
    double strafeToStone;

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
        drive = new Drive(hardwareMap);
        detector = new Detector(hardwareMap);
//        claw = new Claw(hardwareMap);
        state = State.DETECT_STONE;
//        claw.setPositionSync(0.5);
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void loop() {
        switch (state){
            case DETECT_STONE:
                drive.forwardToPositionWait(-10, 0.4);

                int skystone = detector.detectSkystone(500);
                telemetry.addLine("Skystone is " + skystone);
                telemetry.update();
                switch (skystone){
                    case 1:
                        strafeToStone = -2;
                        break;
                    case 2:
                        strafeToStone = 6;
                       break;
                    default:
                        strafeToStone = 14;
                }

                state = State.CAPTURE_FIRST_STONE;

                break;

            case CAPTURE_FIRST_STONE:
                drive.strafeToPositionWait(strafeToStone, 0.4);
                drive.forwardToPositionWait(-15, 0.25);
                drive.forwardToPositionWait(-4, 0.2);

//                claw.setRightClawPositionSync(Claw.CAPTURE);

                state = State.DELIVER_STONE;

                break;

            case DELIVER_STONE:

//                claw.setRightClawPositionSync(0.5);

                if (secondStoneCaptured) {
                    drive.forwardToPositionWait(10, 0.4);
                    drive.strafeToPositionWait(24, 0.35);
                    drive.turnThruAngleWait(-90, 0.35);
                    drive.forwardToPositionWait(-20, 0.6);

                    state = State.MOVE_FOUNDATION;
                } else {
                    drive.forwardToPositionWait(10, 0.4);
                    drive.turnThruAngleWait(-90, 0.35);
                    drive.forwardToPositionWait(-20, 0.6);

                    state = State.STAGE_FOR_SECOND_STONE;
                }

                break;

            case STAGE_FOR_SECOND_STONE:

                drive.forwardToPositionWait(20, 0.6);
                drive.turnThruAngleWait(90, 0.35);
                drive.strafeToPositionWait(strafeToStone+24, 0.4);

                state = State.CAPTURE_SECOND_STONE;

                break;

            case CAPTURE_SECOND_STONE:
                drive.forwardToPositionWait(-15, 0.25);
                drive.forwardToPositionWait(-4, 0.2);

//                claw.setRightClawPositionSync(0.5);

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
                drive.forwardToPositionWait(10, 0.5);

                state = State.STOP;
                break;
            case STOP:
                drive.power(0);
        }
    }

    @Override
    public void stop() {
//        try {
//            FileWriter fileWriter = new FileWriter("/sdcard/FIRST/heading.txt");
//            fileWriter.write(String.format("%.3f", drive.getExternalHeading()));
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
    }
}
