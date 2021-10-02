package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class locaTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot20 R = new Robot20();
        R.init(this, false);

        waitForStart();

        while (!isStopRequested())
        {
            R.updateCurrentPose();
        }
    }
}
