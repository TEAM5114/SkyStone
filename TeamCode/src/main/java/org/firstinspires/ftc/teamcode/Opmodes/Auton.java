
package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name="Autonomous Skeleton", group="")
@Disabled
public class Auton extends LinearOpMode {

    Robot robot = new Robot();


    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            idle();
        }

    }

    void getSkystone(){
//        Use Vuforia to find the skystone and grab it
    }

    void drive() {
//        Drive specified distance at specified heading
    }
}
