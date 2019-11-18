package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

@TeleOp(group = "Comp")
public class StateMachineTeleop extends OpMode {
    MecanumBase drive;
    Claw claw;
    State state;
    double inc = 0.5;
    Vector2d vec = new Vector2d(0,0);
    Trajectory trajectory;
    double clawPositionDown = 0;
    double clawPositionUp = 0;

    enum State {
        DRIVER_RELATIVE,
        ROBOT_RELATIVE,
        PRECISION,
        ESTOP;
    }
    @Override
    public void init() {
        drive = new MecanumREV(hardwareMap);
        claw = new Claw(hardwareMap);
        double globalHeading = 0;
        try {
            BufferedReader bufferedReader = new BufferedReader(new FileReader("/sdcard/FIRST/heading.txt"));
            globalHeading = Double.valueOf(bufferedReader.readLine());
        } catch (Exception e) {
            e.printStackTrace();
        }
        drive.setPoseEstimate(new Pose2d(0,0,globalHeading - Math.PI/2));
        claw.setPositionSync(claw.UP);
        state = State.ROBOT_RELATIVE;
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start(){
    }

    @Override
    public void loop(){
        telemetry.addData("0", state.toString());

        switch (state){
            case DRIVER_RELATIVE:
                if (gamepad1.a){
                    claw.setPosition(claw.UP);
                    state = State.ROBOT_RELATIVE;
                } else if (gamepad1.right_bumper) {
                    claw.setPosition(0.7);
                    state = State.PRECISION;
                } else if (gamepad1.left_bumper) {
                    state = State.ESTOP;
                } else {
                    drive.setDrivePowerDriverRel(new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x));
                    drive.update();
                }
                break;
            case ROBOT_RELATIVE:
                if (gamepad1.a){
                    claw.setPosition(0.5);
                    state = State.DRIVER_RELATIVE;
                } else if (gamepad1.right_bumper){
                    claw.setPosition(0.7);
                    state = State.PRECISION;
                } else if (gamepad1.left_bumper) {
                    state = State.ESTOP;
                } else {
                    drive.setDrivePower(new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x));

                    drive.update();
                }
                break;
            case PRECISION:
                if (gamepad1.a) {
                    claw.setPosition(claw.UP);
                    state = State.ROBOT_RELATIVE;
                } else if (gamepad1.left_bumper) {
                    state = State.ESTOP;
                } else {
                    if (gamepad1.dpad_up){
                        vec = new Vector2d(inc, 0);
                    }
                    if (gamepad1.dpad_down){
                        vec = new Vector2d(-inc, 0);
                    }
                    if (gamepad1.dpad_right){
                        vec = new Vector2d(0, -inc);
                    }
                    if (gamepad1.dpad_left) {
                        vec = new Vector2d(0, inc);
                    }
                    trajectory = drive.trajectoryBuilder()
                            .strafeTo(drive.getPoseEstimate().vec().plus(vec))
                            .build();
                    drive.followTrajectorySync(trajectory);
                }
                break;
            case ESTOP:
                if (gamepad1.a) {
                    claw.setPosition(claw.UP);
                    state = State.ROBOT_RELATIVE;
                } else if (gamepad1.right_bumper){
                    claw.setPosition(0.7);
                    state = State.PRECISION;
                } else {

                }
                break;
        }
        clawPositionDown = gamepad1.right_trigger;
        clawPositionUp = gamepad1.left_trigger;
        if (clawPositionDown != 0) {
            claw.setPositionSync(clawPositionDown);
            clawPositionDown = 0;
        }
        if (clawPositionUp != 0) {
            claw.setPositionSync(1 - clawPositionUp);
            clawPositionUp = 0;
        }
    }

    @Override
    public void stop() {
        drive.setDrivePower(new Pose2d(0,0,drive.getExternalHeading()));
    }
}
