package org.firstinspires.ftc.teamcode.Drive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

@Config
@Autonomous(group = "Tuning")
public class TrackWidthTuner extends LinearOpMode {
    public static double ANGLE = Math.toRadians(180);
    public static int NUM_TRIALS = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumBase drive = new MecanumREV(hardwareMap);

        telemetry.log().add("Press play to begin the trackwidth tuner routine");
        telemetry.log().add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turn(ANGLE);

            while (!isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double trackWidth = DriveConstants.TRACK_WIDTH * ANGLE / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(1000);
        }

        telemetry.log().clear();
        telemetry.log().add("Tuning complete");
        telemetry.log().add(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(),
                trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
