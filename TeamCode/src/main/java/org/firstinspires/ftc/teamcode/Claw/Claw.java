package org.firstinspires.ftc.teamcode.Claw;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@Config
public class Claw {
    public static double UP = 0.0;
    public static double DOWN = 1.0;
    public static double BASE = 0.9;

    private Servo leftServo, rightServo;
    private List<Servo> servos;

    public Claw(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        servos = Arrays.asList(leftServo, rightServo);
    }

    public void setPosition(double position){
//        for (Servo servo : servos){
//            servo.setPosition(position);
//        }
    }
}
