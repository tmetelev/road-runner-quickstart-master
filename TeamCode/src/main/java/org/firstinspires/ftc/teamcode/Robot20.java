package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.drive.Odo;
import org.firstinspires.ftc.teamcode.drive.RobotLocalizer;
import org.firstinspires.ftc.teamcode.util.MotorPower;
import org.firstinspires.ftc.teamcode.util.PIDcontroller;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class Robot20 extends Robot {
    //----------------------------------------Init--------------------------------------------------
    public DcMotor[] wb = new DcMotor[4];
    public final int lf = 0;
    public final int lb = 1;
    public final int rb = 2;
    public final int rf = 3;

    public double current_velocity;  
    public PIDFCoefficients shootcoef;

    public Odo odo;

    public boolean isShooting = false;

    public VoltageSensor voltageSensor;

//    public FileWriter writer;

    public Servo mag_servo, wob_servo;

    public DcMotor encX, encY;
    public DcMotorEx wobble_m, shoot_m, rcatch_m;

    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public Vision vision;
    public OpenCvCamera webcam;
    FtcDashboard dashboard;

    public double shootPrev = 0;

    public double shootdif;

    public double shootPos;

    public double shootvel;

    public double shootTime, tdif, tprev;

//    private String configFileName="FtcLog.txt";

    public int voltage0 = 12;

    ElapsedTime t = new ElapsedTime();
    RobotLocalizer localizer;

    double currentX, currentY, currentHeading;
    double prevX, prevY, prevHeading;

    double prev_left_encoder_pos;
    double prev_right_encoder_pos;
    double prev_center_encoder_pos;
//    public int cnt = 0;

//    double wobbleTunerPos;


    //--------------------------------------Init_methods----------------------------------------------
    @Override
    public void initHWD() {
        wb[lf] = hwd.get(DcMotor.class, "lf");
        wb[lb] = hwd.get(DcMotor.class, "lb");
        wb[rb] = hwd.get(DcMotor.class, "rb");
        wb[rf] = hwd.get(DcMotor.class, "rf");
        mag_servo = hwd.get(Servo.class, "maga");
        wobble_m = hwd.get(DcMotorEx.class, "wobble_m");
        encX = hwd.get(DcMotor.class, "x");
        encY = hwd.get(DcMotor.class, "y");
        shoot_m = hwd.get(DcMotorEx.class, "shoot");
        wob_servo = hwd.get(Servo.class, "wob_s");

//        catcher = hwd.get(DcMotor.class, "catcher");

        wb[rb].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[lf].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[lb].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[rf].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble_m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoot_m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot_m.setDirection(DcMotorSimple.Direction.REVERSE);

//        catcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wb[rf].setDirection(DcMotor.Direction.REVERSE);
        wb[rb].setDirection(DcMotor.Direction.REVERSE);

        wb[lf].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wb[lb].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wb[rb].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wb[rf].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        wobble_m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shoot_m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        voltageSensor = hwd.voltageSensor.iterator().next();

//        catcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hwd.get(BNO055IMU.class, "imu");

        localizer = new RobotLocalizer(hwd, odo);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        odo = new Odo(hwd, tele, imu);

        current_pose = Robot20.current_pose_static;


//        kFs = getMotorVelocityF(SHOOT_MAX_RPM / 60 * SHOOT_TICKS_PER_REV) * 12 / voltageSensor.getVoltage();

//        shootcoef = new PIDFCoefficients(1, 0, 0, 0.5);
//        shoot_m.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shootcoef);
    }

    public double to_square(double n) {
        return (n * n) * (n / Math.abs(n));
    }

    @Override
    public void init(LinearOpMode li, boolean usingRR) {
        this.tele = li.telemetry;
        this.hwd = li.hardwareMap;
        this.li = li;
        initHWD();
//        if (usingRR) {
//            wb[rf].setDirection(DcMotor.Direction.FORWARD);
//            wb[rb].setDirection(DcMotor.Direction.FORWARD);
//        }
        initIMU();
        tele.addData("Init completed", null);
        tele.update();
    }

    public void initIMU() {
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated() && !li.isStopRequested()) {
        }
        tele.addData("Imu calibrated", null);
    }

    public void initOCV() {
        int cameraMonitorViewId = hwd.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwd.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwd.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        vision = new Vision();
        webcam.setPipeline(vision);
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        //if (pos == "R")
        //vision.set_zone_right();
        //else if (pos == "L")
        //vision.set_zone_left();
        delay(1300);
        tele.addData("OCV inited", null);
        tele.update();
    }

    //--------------------------------------Basics--------------------------------------------------
    //----Motors
    public void set_Power(double pwr_lf, double pwr_lb, double pwr_rb, double pwr_rf) {
        wb[lf].setPower(pwr_lf * 0.9);
        wb[lb].setPower(pwr_lb * 0.9);
        wb[rb].setPower(pwr_rb * 0.9);
        wb[rf].setPower(pwr_rf * 0.9);
    }

    //----Angle_read
    public float heading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //--------------------------------------Vision-------------------------------------------------
    @Config
    public static class Vision extends OpenCvPipeline {
        public static int left_x1 = 80;
        public static int left_y1 = 265;
        public static int left_x2 = 130;
        public static int left_y2 = 270;

        public static int right_x1 = 80;
        public static int right_y1 = 35;
        public static int right_x2 = 125;
        public static int right_y2 = 40;

        public static int NUM_OF_ONE = 132; //135
        public static int NUM_OF_FOUR = 147; //142
        int n4 = NUM_OF_FOUR, n1 = NUM_OF_ONE;

        static final Scalar color = new Scalar(255, 0, 255);

        Point top_left;
        Point bottom_right;

        Mat Zone;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        public int orange_pix;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            top_left = new Point(1, 100);
            bottom_right = new Point(1, 100);

            Zone = Cb.submat(new Rect(top_left, bottom_right));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            orange_pix = (int) Core.mean(Zone).val[0];

            Imgproc.rectangle(input, top_left, bottom_right, color, 2);
            return input;
        }

        public void set_zone_left(boolean red) {
            top_left = new Point(left_x1, left_y1);
            bottom_right = new Point(left_x2, left_y2);
            if (red)
                n1 = NUM_OF_ONE + 2;
            Zone = Cb.submat(new Rect(top_left, bottom_right));
        }

        public void set_zone_right(boolean red) {
            top_left = new Point(right_x1, right_y1);
            bottom_right = new Point(right_x2, right_y2);
            if (red)
                n1 = NUM_OF_ONE + 2;
            Zone = Cb.submat(new Rect(top_left, bottom_right));
        }

        public int getNumOfRings() {
            int res = 0;
            if (orange_pix >= n4)
                res = 4;
            else if (orange_pix >= n1)
                res = 1;
            return res;
        }
    }

    //---------------------------------------Wheel_Base---------------------------------------------
    public enum WB_mode {
        CONTROL,
        SOLO_CONTROL,
        STOP,
        AUTO
    }

    public WB_mode mode;

    public Trajectory trajectory = null;

    public boolean field_rotate = false;

    public Thread WheelBase = new Thread() {

        @Override
        public void run() {
            while (!li.isStopRequested()) {
                switch (mode) {
                    case CONTROL:
                        if (field_rotate)
                            WB_control(true, 0.7);
                        else
                            WB_control(false, 0.7);
                        if (Math.abs(opGamepad2.left_stick_y) >= 0.1)
                            encX.setPower(opGamepad2.left_stick_y * zahvat_pwr);
                        else
                            encX.setPower(0);
                        break;
                    case SOLO_CONTROL:
                        WB_control_solo(false, 0.7);
                        if (opGamepad1.dpad_down)
                            encX.setPower(zahvat_pwr);
                        else if (opGamepad1.dpad_up)
                            encX.setPower(-zahvat_pwr);
                        else if (opGamepad1.dpad_right)
                            encX.setPower(0);
                        break;
                    case STOP:
                        set_Power(0, 0, 0, 0);
                        break;
                    case AUTO:
//                        if (driver == null)
//                            driver = new Driver(Robo);
//                        break;
                }
            }
        }
    };

    //----------------------------------------TeleOp-----------------------------------------------
    boolean first_time = true;
    public static double statr_ang = 0;
    public float target_angle = 0;
    public static PIDCoefficients tele_head_coefs = new PIDCoefficients(0.025, 0, 0.002);
    PIDFController tele_heading = new PIDFController(tele_head_coefs);
    public static double k_trig = 0.9;
    public static double k_bmp = 2.6;
    public boolean wb_rotate = false;
    boolean first_rotate = true;

    public void WB_control(boolean field_h, double k) {
        //Initialization
        double sin = Math.sin(Math.toRadians(34));
        double cos = Math.cos(Math.toRadians(34));

        double joyX = opGamepad1.left_stick_x;
        double joyY = -opGamepad1.left_stick_y;
        double trigL = opGamepad1.left_trigger;
        double trigR = opGamepad1.right_trigger;
        boolean bmpL = opGamepad1.left_bumper;
        boolean bmpR = opGamepad1.right_bumper;

        double targetX = 0;
        double targetY = 0;
        double turn = 0;
        double turn1 = 0;
        double Pturn = 0;

        //Modification
        //Turns
        if (!wb_rotate) {
            turn = (trigL - trigR) / 2;
            if (bmpL)
                turn = 0.75;
            if (bmpR)
                turn = -0.75;
            first_rotate = true;
        }
        else {
            if (first_rotate){
                first_rotate = false;
                target_angle = heading();
            }
            if (bmpL)
                turn1 = 1;
            if (bmpR)
                turn1 = -1;
            target_angle += k_trig * (trigL - trigR) + k_bmp * turn1;
            tele_heading.setTargetPosition(target_angle - statr_ang);
            Pturn = tele_heading.update(heading());
        }

        //Preparing
        double len = joyX * joyX + joyY * joyY;
        targetY = (joyY / sin - joyX / cos) / 2;
        targetX = (joyY / sin + joyX / cos) / 2;
        if (opGamepad1.a)
            k = 0.65;
        turn *= k;
        Pturn *= k;
        if (field_h) {
            double ang = heading();
            if (45 + ang + statr_ang < 0)
                ang += 360;
            cos = Math.cos(Math.toRadians(ang + statr_ang));
            sin = Math.sin(Math.toRadians(ang + statr_ang));
            double buf = targetY;
            targetY = cos * targetY - sin * targetX;
            targetX = cos * targetX + sin * buf;
        }


        //Extra_power
        if (Math.abs(targetX) >= Math.abs(targetY) && targetX != 0) {
            targetY /= Math.abs(targetX);
            targetX /= Math.abs(targetX);
        } else if (Math.abs(targetX) <= Math.abs(targetY) && targetY != 0) {
            targetX /= Math.abs(targetY);
            targetY /= Math.abs(targetY);
        }
        targetX *= len * k;
        targetY *= len * k;

        //Output
        if (len < 0.002) {
            set_Power(
                    turn + Pturn,
                    turn + Pturn,
                    -(turn + Pturn),
                    -(turn + Pturn));
            return;
        }
        else {
            set_Power(
                    to_square(targetX + turn + Pturn),
                    to_square(targetY + turn + Pturn),
                    to_square(targetX - turn - Pturn),
                    to_square(targetY - turn - Pturn)
            );
        }
    }

    public void WB_control_solo(boolean field_h, double k) {
        double sin = Math.sin(Math.toRadians(40));
        double cos = Math.cos(Math.toRadians(40));

        double joyX = opGamepad1.left_stick_x;
        double joyY = -opGamepad1.left_stick_y;
        double trigL = opGamepad1.left_trigger;
        double trigR = opGamepad1.right_trigger;

        double targetX = 0;
        double targetY = 0;
        double turn = 0;
        double turn1 = 0;
        double Pturn = 0;

        //Modification
        //Turns
        turn = (trigL - trigR);

        //Preparing
        double len = joyX * joyX + joyY * joyY;
        targetY = (joyY / sin - joyX / cos) / 2;
        targetX = (joyY / sin + joyX / cos) / 2;
        if (opGamepad1.a)
            k = 0.65;

        if (Math.abs(targetX) >= Math.abs(targetY) && targetX != 0) {
            targetY /= Math.abs(targetX);
            targetX /= Math.abs(targetX);
        } else if (Math.abs(targetX) <= Math.abs(targetY) && targetY != 0) {
            targetX /= Math.abs(targetY);
            targetY /= Math.abs(targetY);
        }

        turn *= k;
        targetX *= k;
        targetY *= k;

//        tele.addData("tx", targetX);
//        tele.addData("ty", targetY);

        //Output
        if (len < 0.002) {
            set_Power(
                    turn + Pturn,
                    turn + Pturn,
                    -(turn + Pturn),
                    -(turn + Pturn));
        }
        else {
            set_Power(
                    to_square(targetX + turn + Pturn),
                    to_square(targetY + turn + Pturn),
                    to_square(targetX - turn - Pturn),
                    to_square(targetY - turn - Pturn)
            );
        }
    }

    Pose2d goal_pose_red = new Pose2d(64, -36);
    Pose2d goal_pose_blue = new Pose2d(64, 36);
    Pose2d base_shot_pose_red = new Pose2d(-6, -15);
    Pose2d base_shot_pose_blue = new Pose2d(-6, 45);
    public Pose2d current_pose = new Pose2d(0,0);
    public static Pose2d current_pose_static = new Pose2d(0,0);

    double alpha0B = Math.toDegrees(Math.atan((goal_pose_blue.getY() - base_shot_pose_blue.getY()) / (goal_pose_blue.getX() - base_shot_pose_blue.getX())));
    double alpha0R = Math.toDegrees(Math.atan((goal_pose_red.getY() - base_shot_pose_red.getY()) / (goal_pose_red.getX() - base_shot_pose_red.getX())));

    public double getAngToGoalBlue(){
        double alpha = Math.toDegrees(Math.atan((goal_pose_blue.getY() - current_pose.getY()) / (goal_pose_blue.getX() - current_pose.getX())));
//        tele.addData("a0", alpha0B);
//        tele.addData("a", alpha);
        return alpha - alpha0B;
    }

    public double getAngToGoalRed(){
        double alpha = Math.toDegrees(Math.atan((goal_pose_red.getY() - current_pose.getY()) / (goal_pose_red.getX() - current_pose.getX())));
//        tele.addData("a0", alpha0R);
//        tele.addData("a", alpha);
        return alpha - alpha0B;
    }

    //-------------------------------------Others-------------------------------------------------
    public float correctErrorAngle(float targetA1, float start_ang) {
        float ang = heading() - start_ang;
        float targetA0 = targetA1 - 360;
        float targetA2 = targetA1 + 360;
        float[] targets = {targetA0, targetA1, targetA2};
        float[] deltas = new float[3];
        for (int i = 0; i < 3; i++)
            deltas[i] = targets[i] - ang;
        int k = min(deltas);
        return deltas[k];
    }

    int min(float[] a) {
        int i_min = 0;
        float b_min = a[0];
        for (int i = 0; i < a.length; i++) {
            if (Math.abs(a[i]) < Math.abs(b_min)) {
                b_min = a[i];
                i_min = i;
            }
        }
        return i_min;
    }

    public boolean flag_shoot = false ;
    public boolean target_shoot = false;
    double zahvat_pwr = 0.95;
    public static double servo_close = 0.115;
    public static double servo_copen = 0.34;
    public static int pas0 = 180;

    public void module_control() {
        mag_servo.setPosition(servo_close);
        if (opGamepad1.b) {
            flag_shoot = !flag_shoot;
            target_shoot = false;
            delay(300);
        }
        if (opGamepad1.x) {
            target_shoot = !target_shoot;
            flag_shoot = false;
            delay(300);
        }
        if(opGamepad2.left_bumper)
            wobbleOpen(); //open
        else if (opGamepad2.right_bumper)
            wobbleClose(); //close

       if (opGamepad1.y)
           shoot();
    }

    public void module_control_solo() {
        mag_servo.setPosition(servo_close);
        if (opGamepad1.b) {
            flag_shoot = !flag_shoot;
            target_shoot = false;
            delay(300);
        }
        if (opGamepad1.x) {
            target_shoot = !target_shoot;
            flag_shoot = false;
            delay(300);
        }
        if(opGamepad1.left_bumper)
            wobbleOpen(); //open
        else if (opGamepad1.right_bumper)
            wobbleClose(); //close

        if (opGamepad1.y)
            shoot();
    }

    public void start_shooting(double power) {
        shoot_m.setPower(leveledPower(power));
    }

    public void stop_shooting() {
        shoot_m.setPower(0);
        flag_shoot = false;
    }

    public static double kPs = 0.01;
    public static double kDs = 0.02;
    public static double kFs = 0.58;
    public static PIDCoefficients shoot_coefs = new PIDCoefficients(0.0006, 0, 0.00013);
    public static double target_vel = 1145;
    public static double high_vel = 1290;
    
    public double shoot_vel;

    public double getVelocity() {
        shootPos = shoot_m.getCurrentPosition();
        shootdif = shootPos - shootPrev;
        shootPrev = shoot_m.getCurrentPosition();
        shootTime = t.milliseconds();
        tdif = shootTime - tprev;
        tprev = shootTime;
        return shootdif;
    }

    public Thread ShooterPID = new Thread() {
        @Override
        public void run() {
            PIDFController shoot_controller = new PIDFController(shoot_coefs);
            double current_velocity;
            double output = 0.3;
            ElapsedTime time = new ElapsedTime();
            while (!li.isStopRequested()) {

                if (!flag_shoot && !target_shoot) {
                    shoot_m.setPower(-0.12);
                    output = 0.3;
                }
                else {
                    if (target_shoot)
                        shoot_vel = target_vel;
                    else
                        shoot_vel = high_vel;
                    shoot_controller.setTargetPosition(shoot_vel);
                    current_velocity = shoot_m.getVelocity();
                    output -= shoot_controller.update(current_velocity);
                    shoot_m.setPower(-output);
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("velocity", current_velocity);
                    packet.put("output", output);
                    packet.put("change", shoot_controller.update(current_velocity));
                    packet.put("error", shoot_vel - current_velocity);
                    dashboard.sendTelemetryPacket(packet);
                    delay(10);
                }
            }
        }
    };

    public void shoot() {
        mag_servo.setPosition(servo_copen);
        delay(230);
        mag_servo.setPosition(servo_close);
        delay(pas0);
    }

    public void autoShoot(){
        flag_shoot = true;
        target_shoot = false;
    }

    public void autoTargetShoot(){
        flag_shoot = false;
        target_shoot = true;
    }

    public enum Wobble_mode {
        CONTROL,
        SOLO_CONTROL,
        DOWN,
        DOWN_AUTO,
        UP,
        MID,
        STOP
    }

    public Wobble_mode modeW;

    double WOBBLE_THRESHOLD = 10;

    double start_pose_wobble = 0;
    public static PIDCoefficients wobble_coefs = new PIDCoefficients(0.0025, 0.0002, 0.0001);
    PIDFController wobble_controller = new PIDFController(wobble_coefs);

    public static double wobbleMidPos = 60;
    public static double wobbleUpPos = 240;

    public Thread WobbleControl = new Thread() {
        @Override
        public void run() {
            double err;
            double pwr;
            double wobMidPos = wobbleMidPos;
            double wobUpPos = wobbleUpPos;
            wobble_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobble_m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            start_pose_wobble = wobble_m.getCurrentPosition();
            while (!li.isStopRequested()) {
//                if (flag_shoot)
//                    shoot_m.setPower(shoot_pwr);
//                else
//                    shoot_m.setPower(0);
                TelemetryPacket packet = new TelemetryPacket();
                switch (modeW) {
                    case CONTROL:
                        if (Math.abs(opGamepad2.right_stick_y) >= 0.1)
                            wobble_m.setPower(leveledPower(-opGamepad2.right_stick_y * 6 / voltageSensor.getVoltage()));
                        else
                            wobble_m.setPower(0);
                        break;
                    case SOLO_CONTROL:
                        if (Math.abs(opGamepad1.right_stick_y) >= 0.1)
                            wobble_m.setPower(leveledPower(-opGamepad1.right_stick_y * 6 / voltageSensor.getVoltage()));
                        else
                            wobble_m.setPower(0);
                        break;
                    case DOWN:
                        start_pose_wobble = wobble_m.getCurrentPosition();
                        wobble_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        wobble_m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        double prev_pose;
                        do {
                            wobble_m.setPower(-0.4);
                            prev_pose = wobble_m.getCurrentPosition();
                            delay(100);
                        } while (Math.abs(wobble_m.getCurrentPosition() - prev_pose) >= WOBBLE_THRESHOLD);
                        wobble_m.setPower(0);
                        wobble_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        wobble_m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        start_pose_wobble = wobble_m.getCurrentPosition();
                        wobMidPos = start_pose_wobble + wobbleMidPos;
                        wobUpPos = start_pose_wobble + wobbleUpPos;
                        modeW = Wobble_mode.STOP;
                        break;
                    case DOWN_AUTO:
                        start_pose_wobble = wobble_m.getCurrentPosition();
                        wobble_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        wobble_m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        double prev_pose1;
                        do {
                            wobble_m.setPower(-0.4);
                            prev_pose1 = wobble_m.getCurrentPosition();
                            delay(100);
                        } while (Math.abs(wobble_m.getCurrentPosition() - prev_pose1) >= WOBBLE_THRESHOLD);
                        wobble_m.setPower(0);
                        wobble_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        wobble_m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        start_pose_wobble = wobble_m.getCurrentPosition();
                        wobMidPos = start_pose_wobble + wobbleMidPos;
                        wobUpPos = start_pose_wobble + wobbleUpPos;
                        modeW = Wobble_mode.STOP;
                        break;
                    case MID:
                        wobble_controller.setTargetPosition(wobMidPos);
                        err = (wobble_m.getCurrentPosition() + start_pose_wobble);
                        pwr = wobble_controller.update(err);
                        wobble_m.setPower(leveledPower(pwr));
//                        packet.put("pos", wobble_m.getCurrentPosition());
                        dashboard.sendTelemetryPacket(packet);
                        break;
                    case UP:
                        wobble_controller.setTargetPosition(wobUpPos);
                        err = (wobble_m.getCurrentPosition() + start_pose_wobble);
                        pwr = wobble_controller.update(err);
                        wobble_m.setPower(leveledPower(pwr));
//                        packet.put("pos", wobble_m.getCurrentPosition());
                        dashboard.sendTelemetryPacket(packet);
                        break;
                    case STOP:
                        wobble_m.setPower(0);
                        break;
                }
            }
        }
    };

    double leveledPower(double power) {
        return power + (voltageSensor.getVoltage() - voltage0) / voltage0;
    }

    public void autoTarget() {
        target_shoot = true;
        flag_shoot = true;
        target_vel = 630;
    }

    public void autoHigh() {
        target_shoot = false;
        flag_shoot = true;
    }

    public void stopAutoShoot() {
        flag_shoot = false;
        shoot_m.setPower(0);
    }

    public void setPower(MotorPower power)
    {
        wb[lf].setPower(power.powers[0]);
        wb[lb].setPower(power.powers[1]);
        wb[rb].setPower(power.powers[2]);
        wb[rf].setPower(power.powers[3]);
    }

    double forward_offset = 0;

    public void updateCurrentPose() {
        double left_encoder_pos = localizer.getWheelPositions().get(0);
        double center_encoder_pos = localizer.getWheelPositions().get(1);

        double delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        double delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

        double phi = currentHeading - prevHeading;
        double delta_perp_pos = delta_center_encoder_pos - forward_offset * phi;

        double delta_x = delta_left_encoder_pos * Math.cos(currentHeading) - delta_perp_pos * Math.sin(currentHeading);
        double delta_y = delta_left_encoder_pos * Math.sin(currentHeading) + delta_perp_pos * Math.cos(currentHeading);

        currentX += delta_x;
        currentY += delta_y;
        currentHeading = heading();

        prev_left_encoder_pos = left_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;
        Pose2d currentPose = localizer.getPoseEstimate();
        currentX = currentPose.getX();
        currentY = currentPose.getY();
        currentHeading = currentPose.getHeading();
//        tele.addData("x", currentX);
//        tele.addData("y", currentY);
//        tele.addData("heading", currentHeading);
//        tele.update();
    }

    public static double wob_close = 0.45;
    public static double wob_open = 0.95;
    public void wobbleClose() {
        wob_servo.setPosition(wob_close); //close
    }

    public void wobbleOpen() {
            wob_servo.setPosition(wob_open); //open
    }

    public void wobbleDown() {
        wobble_m.setPower(-0.5);
        delay(500);
        wobble_m.setPower(0);
    }

    public void wobbleUp() {
        wobble_m.setPower(0.5);
        delay(500);
        wobble_m.setPower(0);
    }


    public PIDcontroller controller = new PIDcontroller(kPs, 0, kDs, kFs);


    public Thread ShooterPID2 = new Thread() {
        @Override
        public void run() {
            //controller.isOutputBounded = true;
            controller.maxOutput = 1;
            while (!li.isStopRequested())
            {
                if (!flag_shoot && !target_shoot) {
                    shoot_m.setPower(-0.12);
//                    controller.output = kFs / voltageSensor.getVoltage() * 11.9;
                }
                else {
                    if (target_shoot)
                        controller.target = target_vel;
                    else
                        controller.target = high_vel;
                    controller.kF = kFs * 11.9 / voltageSensor.getVoltage();
                    shoot_vel = shoot_m.getVelocity();
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("velocity", shoot_vel);
                    //tele.addData("velocity", shoot_vel);
                    dashboard.sendTelemetryPacket(packet);
                    shoot_m.setPower(controller.getAction(shoot_vel));
                }
            }
        }
    };

    public Thread odometry = new Thread() {
        @Override
        public void run() {
            while (li.opModeIsActive()){
                odo.update();
                current_pose = new Pose2d(odo.currentPose.getX() + Robot20.current_pose_static.getX(), odo.currentPose.getY() + Robot20.current_pose_static.getY());
            }
        }
    };
//    public Thread velocityCheck = new Thread() {
//        @Override
//        public void run() {
//            while (!li.isStopRequested()) {
//                current_velocity = shoot_m.getVelocity();
//                try {
//                    Thread.sleep(10);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//        }
//    };
}
