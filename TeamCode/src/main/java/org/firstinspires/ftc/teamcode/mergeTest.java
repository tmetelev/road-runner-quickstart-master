package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.MotorPower;
import org.firstinspires.ftc.teamcode.util.WB;

public class  mergeTest extends LinearOpMode {
    MotorPower translational = new MotorPower(1, 1, 1, 1);
    MotorPower rotational = new MotorPower(1, 1, -1, -1);
    MotorPower merged = WB.merge(translational, rotational);
    @Override
    public void runOpMode() throws InterruptedException {
        Robot20 R = new Robot20();
        R.init(this, false);

        waitForStart();

        while (!isStopRequested())
            R.setPower(merged);
    }
}
