package org.firstinspires.ftc.teamcode.Drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Drive.DriveConstants.kV;

@Config
@Autonomous(group = "Tuning")
public class DriveVelocityPIDTuner extends LinearOpMode {
    public static double DISTANCE = 72;

    private static final String PID_VAR_NAME = "VELO_PID";

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private String catName;
    private CustomVariable catVar;

    private MecanumBase drive;

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                DriveConstants.BASE_CONSTRAINTS.maxVel,
                DriveConstants.BASE_CONSTRAINTS.maxAccel,
                DriveConstants.BASE_CONSTRAINTS.maxJerk);
    }

    private void addPidVariable(){
        catName = getClass().getSimpleName();
        catVar = (CustomVariable) dashboard.getConfigRoot().getVariable(catName);
        if (catVar == null){
            catVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(catName, catVar);

            RobotLog.w("Unable to find top-level category %s", catName);
        }

        CustomVariable pidVar = new CustomVariable();
        pidVar.putVariable("kP", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kP;
            }

            @Override
            public void set(Double value) {
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(coeffs.kP, value, coeffs.kD));
            }
        }));
        pidVar.putVariable("kD", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kD;
            }

            @Override
            public void set(Double value){
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(coeffs.kP, coeffs.kI, value));
            }
        }));

        catVar.putVariable(PID_VAR_NAME, pidVar);
        dashboard.updateConfig();
    }

    private void removePidVariable(){
        if (catVar.size() > 1){
            catVar.removeVariable(PID_VAR_NAME);
        } else {
            dashboard.getConfigRoot().removeVariable(catName);
        }
        dashboard.updateConfig();
    }

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new MecanumREV(hardwareMap);

        addPidVariable();

        NanoClock clock = NanoClock.system();

        telemetry.log().add("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();

        while (!isStopRequested()){
            double profileTime = clock.seconds() - profileStart;

            if (profileTime > activeProfile.duration()){
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = kV * motionState.getV();
            drive.setDrivePower(new Pose2d(targetPower, 0, 0));

            List<Double> velocities = drive.getWheelVelocities();

            telemetry.addData("targetVelocity", motionState.getV());
            for (int i = 0; i < velocities.size(); i++){
                telemetry.addData("velocity" + i, velocities.get(i));
                telemetry.addData("error" + i, motionState.getV() - velocities.get(i));
            }
            telemetry.update();
        }

        removePidVariable();
    }
}