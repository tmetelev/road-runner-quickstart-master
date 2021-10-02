package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot20;
import org.firstinspires.ftc.teamcode.vision.ContourRingPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ContourRingPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(group = "r")
public class RedWallAuto extends LinearOpMode {
    Robot20 R = new Robot20();
    public static double shootangle = 13;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam
    public static double pp = 140.00;

    private ContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);

        ContourRingPipeline.Height height;

        Pose2d startPose = new Pose2d(-60, -48, 0);

        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new ContourRingPipeline(telemetry, DEBUG));

        ContourRingPipeline.CAMERA_WIDTH = CAMERA_WIDTH;

        ContourRingPipeline.HORIZON = HORIZON;

//        ContourRingPipeline.lowerOrange = new Scalar(0.0, pp, 0.0);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera, 30);
        dashboard.setTelemetryTransmissionInterval(500);

        R.delay(3000);

        while (!isStarted()) {
            height = pipeline.height;
            telemetry.addData("rings", "" + height);
            telemetry.update();
        }

        height = pipeline.height;
        telemetry.addData("rings", "" + height);
        telemetry.update(); 

        drive.setPoseEstimate(startPose);
        R.ShooterPID2.start();
        R.WobbleControl.start();
        R.wobbleOpen();
        R.modeW = Robot20.Wobble_mode.STOP;
        R.mag_servo.setPosition(0.09);

        switch (height) {
            case ZERO:
                Trajectory trajectoryDown = drive.trajectoryBuilder(startPose, 0)
                        .splineToSplineHeading(new Pose2d(-0, -60, 0), 0)
                        .build();

                Trajectory downtoshoot = drive.trajectoryBuilder(trajectoryDown.end())
                        .lineToLinearHeading(new Pose2d(-6, -50 , Math.toRadians(0)))
                        .build();

//                Trajectory downToWob = drive.trajectoryBuilder(downtoshoot.end(), true)
//                        .lineToLinearHeading(new Pose2d(-25, -27, Math.toRadians(0)))
//                        .build();
//
//                Trajectory downToWob2 = drive.trajectoryBuilder(downToWob.end(), true)
//                        .lineToLinearHeading(new Pose2d(12, -48, Math.toRadians(90)))
//                        .build();

                Trajectory back = drive.trajectoryBuilder(downtoshoot.end())
                        .lineToLinearHeading(new Pose2d(-24, -53 , Math.toRadians(0)))
                        .build();

                Trajectory downpark = drive.trajectoryBuilder(back.end())
                        .lineToLinearHeading(new Pose2d(12,-36, 0))
                        .build();

                drive.followTrajectory(trajectoryDown);
                R.autoHigh();
                Robot20.high_vel = 1350;
                drive.followTrajectory(downtoshoot);
                drive.turn(Math.toRadians(20));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.shoot();
                R.stopAutoShoot();
                drive.followTrajectory(back);
                R.delay(17300);
//                R.delay(5000);
//                drive.followTrajectory(downToWob);
//                Robot20.high_vel = 1300;
//                R.modeW = Robot20.Wobble_mode.DOWN;
//                R.delay(1000);
//                back(10, 0.3, drive);
//                R.wobbleClose();
//                R.delay(1000);
//                R.modeW = Robot20.Wobble_mode.UP;
//                drive.followTrajectory(downToWob2);
//                R.modeW = Robot20.Wobble_mode.DOWN_AUTO;
//                R.wobbleOpen();
//                R.delay(1000);
                drive.followTrajectory(downpark);
//                R.modeW = Robot20.Wobble_mode.UP;
                Robot20.statr_ang = R.heading();
                break;
            case ONE:
                Trajectory trajectoryMid = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, -51, 0), 0)
                        .splineToSplineHeading(new Pose2d(24, -42, Math.toRadians(30)), Math.toRadians(30))
                        .build();

                Trajectory midtoshoot = drive.trajectoryBuilder(trajectoryMid.end())
                        .lineToLinearHeading(new Pose2d(-5 , -52, Math.toRadians(0)))
                        .build();

                Trajectory midtozah = drive.trajectoryBuilder(midtoshoot.end())
                        .lineToLinearHeading(new Pose2d(-5 , -35, 0))
                        .build();

                Trajectory midtoshoot2 = drive.trajectoryBuilder(midtozah.end())
                        .lineToLinearHeading(new Pose2d(-8 , -52, Math.toRadians(0)))
                        .build();

//                Trajectory midToWob = drive.trajectoryBuilder(midtoshoot2.end(), true)
//                        .lineToLinearHeading(new Pose2d(-25, -26, Math.toRadians(0)))
//                        .build();
//
//                Trajectory midToWob2 = drive.trajectoryBuilder(midToWob.end(), true)
//                        .lineToLinearHeading(new Pose2d(15, -44, Math.toRadians(-180)))
//                        .build();

                Trajectory midpark = drive.trajectoryBuilder(midtoshoot2.end())
                        .lineToLinearHeading(new Pose2d(12, -58, 0))
                        .build();

                drive.followTrajectory(trajectoryMid);
                R.autoHigh();
                Robot20.high_vel = 1350;
                drive.followTrajectory(midtoshoot);
                drive.turn(Math.toRadians(17));
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();

//                drive.followTrajectory(midtozah);
//                R.encX.setPower(1);
//                R.delay(20);
//                back(10, 0.3, drive);
//                R.delay(20);
//                forward(10, 0.3, drive);
//                drive.followTrajectory(midtoshoot2);
//                drive.turn(Math.toRadians(17));
//                R.encX.setPower(0);
//                R.delay(20);
//                R.shoot();
//                R.delay(20);
//                R.stop_shooting();
//                drive.followTrajectory(midToWob);
//                Robot20.high_vel = 1300;
//                R.modeW = Robot20.Wobble_mode.DOWN;
//                R.delay(1000);
//                back(10, 0.3, drive);
//                R.wobbleClose();
//                R.delay(1000);
//                R.modeW = Robot20.Wobble_mode.UP;
//                drive.followTrajectory(midToWob2);
//                R.modeW = Robot20.Wobble_mode.DOWN_AUTO;
//                R.wobbleOpen();
//                R.delay(1000);
//                R.modeW = Robot20.Wobble_mode.UP;

                drive.followTrajectory(midpark);
                R.stop_shooting();
                Robot20.statr_ang = R.heading();
                break;
            case FOUR:
                Trajectory trajectoryHigh = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, -51, 0), 0)
                        .splineToSplineHeading(new Pose2d(48, -60, 0), 0)
                        .build();

                Trajectory hightoshoot = drive.trajectoryBuilder(trajectoryHigh.end())
                        .lineToLinearHeading(new Pose2d(-3, -54, Math.toRadians(0)))
                        .build();

                Trajectory hightozah = drive.trajectoryBuilder(hightoshoot.end())
                        .lineToLinearHeading(new Pose2d(-5 , -32, 0))
                        .build();

                Trajectory hightozah1 = drive.trajectoryBuilder(new Pose2d(-17, -32.5))
                        .lineToLinearHeading(new Pose2d(-5 , -32.5, 0))
                        .build();

                Trajectory hightozah2 = drive.trajectoryBuilder(new Pose2d(-28, -32.5))
                        .lineToLinearHeading(new Pose2d(-3 , -32.5, 0))
                        .build();

                Trajectory highpark = drive.trajectoryBuilder(hightoshoot
                        .end())
                        .lineToLinearHeading(new Pose2d(10, -58, 0))
                        .build();

                ElapsedTime t = new ElapsedTime();
                double t0 = t.milliseconds();
                Robot20.high_vel = 1350;
                drive.followTrajectory(trajectoryHigh);
                R.autoHigh();
                drive.followTrajectory(hightoshoot);
                drive.turn(Math.toRadians(20));
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.shoot();
//                drive.followTrajectory(hightozah);
//                drive.turn(Math.toRadians(0 - R.heading()));
//                //R.encX.setPower(-1);
//                R.encX.setPower(0.95);
//                back(7.5, 0.2, drive);
//                forward(0.1, 0.2, drive);
//                R.delay(500);
//                back(1.5, 0.2, drive);
//                forward(0.1, 0.2, drive);
//                R.delay(500);
//                back(2, 0.2, drive);
//                forward(0.1, 0.2, drive);
//                R.high_vel = 1300;
//                drive.followTrajectory(hightozah1);
//                drive.turn(Math.toRadians(7));
//                R.shoot();
//                R.delay(100);
//                R.shoot();
//                R.delay(100);
//                R.shoot();
//                drive.turn(Math.toRadians(0 - R.heading()));
//                if (t.milliseconds() - t0 <= 25000){
//                    telemetry.addData("Go to 4th", null);
//                    back(20, 0.3, drive);
//                    if (t.milliseconds() - t0 <= 27000) {
//                        drive.followTrajectory(hightozah2);
//                        drive.turn(Math.toRadians(5));
//                        R.encX.setPower(0);
//                        R.shoot();
//                        R.delay(100);
//                        R.shoot();
//                    }
//                }
//                telemetry.addData("Go to park", null);
//                telemetry.update();
                R.stopAutoShoot();
                drive.followTrajectory(highpark);
                Robot20.statr_ang = R.heading();
                break;
        }
        R.ShooterPID2.interrupt();
        R.WobbleControl.interrupt();
        Robot20.statr_ang = R.heading();
        Robot20.current_pose_static = drive.currentPose;
        Robot20.high_vel = 1330;
    }

    void back (double x, double p, SampleMecanumDrive drive){
        drive.update();
        x = drive.currentPose.getX() - x;
        while (drive.currentPose.getX() >= x){
            R.set_Power(-p, -p, -p, -p);
            drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();
    }
    void forward (double x, double p, SampleMecanumDrive drive){
        drive.update();
        x = drive.currentPose.getX() + x;
        while (drive.currentPose.getX() <= x){
            R.set_Power(p, p, p, p);
            drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();
    }
}
