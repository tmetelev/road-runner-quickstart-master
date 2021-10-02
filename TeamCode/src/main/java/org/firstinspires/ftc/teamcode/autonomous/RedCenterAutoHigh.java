package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot20;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ContourRingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

@Config
@Autonomous(group = "r")
public class RedCenterAutoHigh extends LinearOpMode {
    Robot20 R = new Robot20();
    public static double shootangle = -10;
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

        Pose2d startPose = new Pose2d(-60, -24, 0);

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
        R.WobbleControl.start();
        R.modeW = Robot20.Wobble_mode.STOP;

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
        R.wobbleOpen();
        R.mag_servo.setPosition(0.1);

        switch (height) {
            case ZERO:
                Trajectory trajectoryDown = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-48, -24), 0)
                        .splineToSplineHeading(new Pose2d(3, -48, Math.toRadians(-40)), Math.toRadians(-40))
                        .build();

                Trajectory downtoshoot = drive.trajectoryBuilder(trajectoryDown.end())
                        .lineToLinearHeading(new Pose2d(-4, -26, Math.toRadians(0)))
                        .build();

                Trajectory downpark = drive.trajectoryBuilder(downtoshoot.end())
                        .lineToLinearHeading(new Pose2d(12, -26, 0))
                        .build();

                Trajectory downToWob = drive.trajectoryBuilder(downtoshoot.end())
                        .lineToLinearHeading(new Pose2d(-30, -51, 0))
                        .build();

                Trajectory downToWob2 = drive.trajectoryBuilder(downToWob.end())
                        .lineToLinearHeading(new Pose2d(14, -36, 0))
                        .build();

                Trajectory downToWob4 = drive.trajectoryBuilder(downToWob2.end().plus(new Pose2d(0, -4, Math.toRadians(90))))
                        .lineToLinearHeading(downToWob2.end().plus(new Pose2d(0, 5, Math.toRadians(90))))
                        .build();

                R.autoHigh();
                drive.followTrajectory(trajectoryDown);

//                Robot20.high_vel = 710;
                drive.followTrajectory(downtoshoot);
                drive.turn(Math.toRadians(-R.heading()));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.stop_shooting();


                R.modeW = Robot20.Wobble_mode.DOWN_AUTO;
                drive.followTrajectory(downToWob);
                drive.turn(Math.toRadians(-R.heading()));
//                R.delay(1000);
//                back(10, 0.3, drive);
                R.wobbleClose();
                R.delay(500);
                R.modeW = Robot20.Wobble_mode.UP;
//                R.delay(2000);
                drive.followTrajectory(downToWob2);
                double dang = 90 - R.heading();
                drive.turn(Math.toRadians(dang));
                double p = 0.4;
                R.set_Power(-p, -p, -p, -p);
                R.delay(700);
                R.set_Power(0, 0, 0, 0);

                R.modeW = Robot20.Wobble_mode.DOWN_AUTO;
                R.delay(650);
                R.wobbleOpen();
                R.set_Power(p, p, p, p);
                R.delay(500);
                R.set_Power(0, 0, 0, 0);

//                Robot20.high_vel = 700;
               // drive.followTrajectory(downpark);
                break;
            case ONE:
                Trajectory trajectoryMid = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, -23, 0), 0)
                        .splineToSplineHeading(new Pose2d(24, -30, Math.toRadians(-15)), Math.toRadians(-15))
                        .build();

                Trajectory midtoshoot = drive.trajectoryBuilder(trajectoryMid.end())
                        .lineToLinearHeading(new Pose2d(-4, -27, Math.toRadians(0)))
                        .build();

                Trajectory midtozah = drive.trajectoryBuilder(midtoshoot.end())
                        .lineToLinearHeading(new Pose2d(-5 , -36, 0))
                        .build();

                Trajectory midtoshoot2 = drive.trajectoryBuilder(midtozah.end())
                        .lineToLinearHeading(new Pose2d(-3 , -28, Math.toRadians(0)))
                        .build();

                Trajectory midToWob = drive.trajectoryBuilder(midtoshoot.end())
                        .lineToLinearHeading(new Pose2d(-31, -51, 0))
                        .build();

                Trajectory midToWob2 = drive.trajectoryBuilder(midToWob.end())
                        .lineToLinearHeading(new Pose2d(14, -39, 0))
                        .build();

                Trajectory midpark = drive.trajectoryBuilder(midtoshoot.end())
                        .lineToLinearHeading(new Pose2d(12, -24, 0))
                        .build();
                R.autoHigh();
                drive.followTrajectory(trajectoryMid);
                drive.followTrajectory(midtoshoot);
                drive.turn(Math.toRadians(-R.heading()));

                //R.delay(500);
                R.shoot();
               // R.delay(100);
                R.shoot();
               // R.delay(100);
                R.shoot();
                //R.delay(100);
                drive.followTrajectory(midtozah);
                R.encX.setPower(1);
                R.delay(20);
                back(10, 0.3, drive);
                R.delay(20);
                forward(10, 0.3, drive);
                drive.followTrajectory(midtoshoot2);
                drive.turn(Math.toRadians(0 - R.heading()));

//                drive.turn(Math.toRadians(-10));
                R.encX.setPower(0);
                R.delay(200);
                R.shoot();
                //R.delay(1000);
                R.stop_shooting();

                R.modeW = Robot20.Wobble_mode.DOWN;
                drive.followTrajectory(midToWob);
                drive.turn(Math.toRadians(-R.heading()));
//                R.delay(1000);
//                back(10, 0.3, drive);
                R.wobbleClose();
                R.delay(500);
                R.modeW = Robot20.Wobble_mode.UP;
//                R.delay(2000);
                drive.followTrajectory(midToWob2);
                double dang1 = 180 - R.heading();
                double p1 = 0.4;
                drive.turn(Math.toRadians(dang1));
                R.set_Power(-p1, -p1, -p1, -p1);
                R.delay(700);
                R.set_Power(0, 0, 0, 0);

                R.modeW = Robot20.Wobble_mode.DOWN_AUTO;
                R.delay(650);
                R.wobbleOpen();
                R.set_Power(p1, p1, p1, p1);
                R.delay(500);
                R.set_Power(0, 0, 0, 0);
//                drive.turn(Math.toRadians(10));
//                Robot20.high_vel = 700;
                //drive.followTrajectory(midpark);
                break;
            case FOUR:
                Trajectory trajectoryHigh = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, -23, 0), 0)
                        .splineToSplineHeading(new Pose2d(48, -48, Math.toRadians(-45)), Math.toRadians(-45))
                        .build();

                Trajectory hightoshoot = drive.trajectoryBuilder(trajectoryHigh.end())
                        .lineToLinearHeading(new Pose2d(-4, -25, Math.toRadians(0)))
                        .build();

                Trajectory hightozah = drive.trajectoryBuilder(hightoshoot.end())
                        .lineToLinearHeading(new Pose2d(-4 , -36, 0))
                        .build();

                Trajectory hightozah1 = drive.trajectoryBuilder(new Pose2d(-22, -35))
                        .lineToLinearHeading(new Pose2d(-4, -35, 0))
                        .build();

                Trajectory hightozah2 = drive.trajectoryBuilder(new Pose2d(-24, -35))
                        .lineToLinearHeading(new Pose2d(-4 , -35, 0))
                        .build();

                Trajectory hightozah3 = drive.trajectoryBuilder(new Pose2d(-4, -34, 0))
                        .lineToLinearHeading(new Pose2d(-4, -35, 0))
                        .build();

                Trajectory highpark = drive.trajectoryBuilder(hightozah2.end())
                        .lineToLinearHeading(new Pose2d(12, -50, 0))
                        .build();

                ElapsedTime t = new ElapsedTime();
                double t0 = t.milliseconds();
                drive.followTrajectory(trajectoryHigh);
                R.autoHigh();
                drive.followTrajectory(hightoshoot);
                drive.turn(Math.toRadians(-R.heading()));

                //back(2, 0.44, drive);
                //R.delay(500);
                R.shoot();
                //R.delay(100);
                R.shoot();
                //R.delay(100);
                R.shoot();
                //R.delay(100);
                drive.followTrajectory(hightozah);
                R.encX.setPower(0.95) ;
                back(8, 0.2, drive);
                forward(1, 0.3, drive);
                R.delay(500);
                back(5, 0.3, drive);
                forward(1, 0.3, drive);
                R.delay(500);
                back(5, 0.2, drive);
                R.delay(500);
                drive.followTrajectory(hightozah1);
//                drive.turn(Math.toRadians(-10));
                drive.turn(Math.toRadians(6 - R.heading()));
                //R.delay(500);
                R.shoot();
                R.shoot();
                R.shoot();
                R.delay(300);
                drive.turn(Math.toRadians(8 - R.heading()));
//                drive.turn(Math.toRadians(10));
                if (t.milliseconds() - t0 <= 24000){
                    telemetry.addData("Go to 4th", null);
                    drive.followTrajectory(hightozah3);
                    back(20, 0.3, drive);
                    if (t.milliseconds() - t0 <= 26000) {
                        drive.followTrajectory(hightozah2);
//                        drive.turn(Math.toRadians(-10));
                        R.encX.setPower(0);
                        drive.turn(Math.toRadians(6 - R.heading()));
                        R.shoot();
                        R.shoot();
                    }
                }
                R.stop_shooting();
                drive.followTrajectory(highpark);
                R.modeW = Robot20.Wobble_mode.DOWN;
                R.wobbleOpen();
//                Robot20.high_vel = 700;
                break;
        }
        R.ShooterPID2.interrupt();
        Robot20.statr_ang = R.heading();
        Robot20.current_pose_static = drive.currentPose;
    }

    void back (double x, double p, SampleMecanumDrive drive){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        drive.update();
        x = drive.currentPose.getX() - x;
        while (drive.currentPose.getX() >= x && t.milliseconds() - t0 < 5000){
            R.set_Power(-p, -p, -p, -p);
            drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();
    }
    void forward (double x, double p, SampleMecanumDrive drive){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        drive.update();
        x = drive.currentPose.getX() + x;
        while (drive.currentPose.getX() <= x && t.milliseconds() - t0 < 5000){
            R.set_Power(p, p, p, p);
            drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();
    }

    void strafe (double x, double p, SampleMecanumDrive drive){
        //R.WheelBase.interrupt();
                    drive.update();
        x = R.current_pose.getY() - x;
        while (drive.currentPose.getY() >= x){
            R.set_Power(-p, -p, -p, -p);
            drive.update();

            //drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();

//        rotate(-R.heading(), 0.4);
        //R.WheelBase.start();
    }

    void strafeL (double x, double p, SampleMecanumDrive drive){
        //R.WheelBase.interrupt();
        drive.update();
        x = R.current_pose.getY() + x;
        while (drive.currentPose.getY() <= x){
            R.set_Power(p, p, p, p);
            drive.update();

            //drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();

//        rotate(-R.heading(), 0.4);
        //R.WheelBase.start();
    }

    void rotateL (double x, double p){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (R.heading() <= x && t.milliseconds() - t0 < 5000){
            R.set_Power(p, p, -p, -p);
        }
        R.set_Power(0, 0, 0, 0);
    }

    void rotateR (double x, double p){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (R.heading() >= x && t.milliseconds() - t0 < 5000){
            R.set_Power(-p, -p, p, p);
        }
        R.set_Power(0, 0, 0, 0);
    }

    void to_coord(double x, double y, double p, SampleMecanumDrive drive)  {    }
}