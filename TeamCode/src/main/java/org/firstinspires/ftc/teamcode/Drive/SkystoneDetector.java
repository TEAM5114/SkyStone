package org.firstinspires.ftc.teamcode.Drive;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SkystoneDetector {
    private TFObjectDetector tfod;


    public SkystoneDetector(TFObjectDetector tfod) {
        this.tfod = tfod;
        tfod.activate();
    }

    public void deactivate(){
        tfod.deactivate();
    }

    public List<Recognition> detectSkystone(){
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        return updatedRecognitions;
//            telemetry.addData("# Object Detected", updatedRecognitions.size());
//            // step through the list of recognitions and display boundary info.
//            int i = 0;
//            for (Recognition recognition : updatedRecognitions) {
//                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                        recognition.getLeft(), recognition.getTop());
//                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                        recognition.getRight(), recognition.getBottom());
//            }
//            telemetry.update();
//        }
    }
}
