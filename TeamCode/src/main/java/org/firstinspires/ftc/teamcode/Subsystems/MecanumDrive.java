package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.HardwareSkyStone;

public class MecanumDrive {
    HardwareSkyStone hw;
    Orientation angles;
    double orientation;
    double angle;
    double leftFrontPower;
    double leftRearPower;
    double rightFrontPower;
    double rightRearPower;
    double scale;

    //Constructor
    public MecanumDrive(HardwareSkyStone hardware){
        hw = hardware;
    }

    public void run(double magnitude, double heading, double pivot, boolean DRIVER_RELATIVE){
        if (DRIVER_RELATIVE) {
            orientation = getOrientation();
        } else {
            orientation = 0;
        }
        angle = heading - orientation;

        leftFrontPower = magnitude * Math.cos(angle - Math.PI/4) + pivot;
        leftRearPower = magnitude * Math.sin(angle - Math.PI/4) + pivot;
        rightFrontPower = magnitude * Math.sin(angle - Math.PI/4) - pivot;
        rightFrontPower = magnitude * Math.cos(angle - Math.PI/4) - pivot;

        scale = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower))
        );

        if (scale > 1){
            leftFrontPower /= scale;
            leftRearPower /= scale;
            rightFrontPower /= scale;
            rightRearPower /= scale;
        }

        hw.leftFrontDrive.setPower(leftFrontPower);
        hw.leftRearDrive.setPower(leftRearPower);
        hw.rightFrontDrive.setPower(rightFrontPower);
        hw.rightRearDrive.setPower(rightRearPower);
    }

    public void pivotToAngle(double power, double angle){
        orientation = getOrientation();

        double target = orientation + angle;
        while (getOrientation() < target){
            run(0,0, power, false);
        }
    }

    private double getOrientation(){
        return hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

}
