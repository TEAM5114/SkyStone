package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name = "Vuforia Test", group = "Test")
@Disabled
public class VuforiaTest extends LinearOpMode {
    OpenGLMatrix skystoneLocation;
    private Robot robot = new Robot();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            if (robot.skystoneIsVisible()){
                telemetry.addLine("Skystone is visible!!!");
//                skystoneLocation = robot.getSkystoneLocation();
//                if (skystoneLocation != null) {
//                    VectorF translation = skystoneLocation.getTranslation();
//                    telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                            translation.get(0), translation.get(1), translation.get(2));
//
//                    // express the rotation of the robot in degrees.
//                    Orientation rotation = Orientation.getOrientation(skystoneLocation, EXTRINSIC, XYZ, DEGREES);
//                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//                }
            } else {
                telemetry.addData("Skystone not visible", "");
            }
            telemetry.update();
        }
    }

}
