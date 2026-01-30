package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sorter {

    private Servo sorter;

    private final double step = 1.0 / 15.0;
    private double position = 0.5;

    private boolean lastLeft = false;
    private boolean lastRight = false;

    public void init(HardwareMap hwMap) {
        sorter = hwMap.get(Servo.class, "sorter");
        sorter.setPosition(position);
    }

    public void rotateSteps(int n) {
        position += step * n;

        if (position > 1.0) position = 1.0;
        if (position < 0.0) position = 0.0;

        sorter.setPosition(position);
    }

    public void update(boolean dpadLeft, boolean dpadRight) {

        if (dpadRight && !lastRight) {
            rotateSteps(1);
        }

        if (dpadLeft && !lastLeft) {
            rotateSteps(-1);
        }

        lastLeft = dpadLeft;
        lastRight = dpadRight;
    }

    public void setRawPosition(double pos) {
        if (pos > 1.0) pos = 1.0;
        if (pos < 0.0) pos = 0.0;
        position = pos;
        sorter.setPosition(position);
    }

    public double getPosition() {
        return position;
    }
}
