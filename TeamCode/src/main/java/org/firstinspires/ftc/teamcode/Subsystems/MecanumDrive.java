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
        
        
        //need to check if we get full power on the robot wheels after this transformation. 
        //if the driver asks for full power and then we adjust with orientation we might need to recalculate the magnitude first.
        //the PS4 joystick aren't radial.   So the magnitude at 45 degrees on the stick is 1.41 at 30 is only 1.15
        // if the stick is at 30 and the orientation adds another 15 degrees we are short on the intended magnitude.
        // I think we need to calculate the percent of full scale at the joystick angle and then use that as the magnitude at the
        // corrected angle.
        // -  to test is this is an issue we just need to display the motor power while test drivinig.
        

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
        
        
        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Left Rear Power" , leftRearPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Right Rear Power", rightRearPower);
        updateTelemetry(telemetry);
                          
        
        
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
