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

        rightServo.setDirection(Servo.Direction.REVERSE);

        servos = Arrays.asList(leftServo, rightServo);
    }

    public void setPosition(double position){
        for (Servo servo : servos){
            servo.setPosition(position);
        }
    }

    public void setPositionSync(double position){
        setPosition(position);
        for (Servo servo : servos){
            while (servo.getPosition() != position){
                continue;
            }
        }
    }

    public void setRightClawPosition(double position){
        rightServo.setPosition(position);
    }

    public void setRightClawPositionSync(double postion){
        setRightClawPosition(postion);
        while (rightServo.getPosition() != postion){continue;}
    }

    public void setLeftClawPosition(double position){
        leftServo.setPosition(position);
    }

    public void setLeftClawPositionSync(double positon){
        setLeftClawPosition(positon);
        while (leftServo.getPosition() != positon){continue;}
    }
}
