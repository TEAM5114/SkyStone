package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Claw.Claw;
import org.firstinspires.ftc.teamcode.Drive.MecanumBase;
import org.firstinspires.ftc.teamcode.Drive.MecanumREV;

@TeleOp
public class StateMachineTeleop extends OpMode {
    MecanumBase drive;
    Claw claw;
    State state;

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
        drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){
        state = State.ROBOT_RELATIVE;
    }

    @Override
    public void loop(){
        telemetry.addData("0", state.toString());

        switch (state){
            case DRIVER_RELATIVE:
                if (gamepad1.a){
                    claw.setPosition(0.5);
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
                    claw.setPosition(0);
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
                    claw.setPosition(0.5);
                    state = State.ROBOT_RELATIVE;
                } else if (gamepad1.left_bumper) {
                    state = State.ESTOP;
                } else {

                }
                break;
            case ESTOP:
                if (gamepad1.a) {
                    claw.setPosition(0.5);
                    state = State.ROBOT_RELATIVE;
                } else if (gamepad1.right_bumper){
                    claw.setPosition(0.7);
                    state = State.PRECISION;
                } else {

                }
                break;
        }
        if (gamepad1.right_trigger > 0) {
            claw.setPosition(1);
        }
        if (gamepad1.left_trigger > 0) {
            claw.setPosition(0);
        }

    }

    @Override
    public void stop() {
        drive.setDrivePower(new Pose2d(0,0,drive.getExternalHeading()));
    }
}
