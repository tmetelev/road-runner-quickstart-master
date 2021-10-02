package org.firstinspires.ftc.teamcode.util;

public class MotorPower {
    public double[] powers;

    public MotorPower(double leftFront, double leftRear, double rightRear, double rightFront) {
        this.powers[0] = leftFront;
        this.powers[1] = leftRear;
        this.powers[2] = rightRear;
        this.powers[3] = rightFront;
    }
}
