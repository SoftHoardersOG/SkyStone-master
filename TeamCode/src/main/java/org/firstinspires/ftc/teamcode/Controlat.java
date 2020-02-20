package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;


@TeleOp(name = "Controlat", group = "Linear OpMode")

public class Controlat extends LinearOpMode {

    private Hardware robot = new Hardware();

    private double pow2 = 0;
    private int n2 = 0;
    private int etaj = 0;
    private int etajMaxim = 1; // to be : 13

    boolean cap;

    boolean reset = false;

    private double inc1 = 0;
    private double inc2 = 0;

    private double distantaCub = 190;  //// init!!!!  in mm!!!
    private double dist2 = 60; /// in mm!!!!
    private int ct = 0;

    private int[] initialg = {-900, -1200, -1550, -1900, -450, -850, -1200,-1500,-1850,-2200,-2400,0,0};
    private int[] rotatie = {1500, 1500, 1500, 1500, 600, 600, 600,600,600,600,600,0,0};//inainte 600
    private int[] finalg = {-700, -950, -1200, -1550, -300, -650, -850,-1200,-1650,-1900,-2200,0,0};
    // {-600, -900, -1150, -1550, -300, -600, -850,-1200,-1600,-1900,-2200,0,0} inINTE
    private ElapsedTime timedecr = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime time2 = new ElapsedTime();
    private ElapsedTime time3 = new ElapsedTime();


    public void Drive() {
        if (robot.front_right == null || robot.front_left == null || robot.back_right == null || robot.back_left == null)
            return;

        double r = Math.hypot(-gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        robot.front_left.setPower(-v1);
        robot.front_right.setPower(-v2);
        robot.back_left.setPower(-v3);
        robot.back_right.setPower(-v4);
    }


    public void Returneaza() {
        if(robot.rotatie == null || robot.glisisus == null)
            return;
        final int ridicare = -470;
        robot.brat.setPosition(0.2);
        if (etaj > -1) {
            int poz_init = robot.glisisus.getCurrentPosition();
            sleep(200); ///!!! de facut cu time elapsed// ba minele adu-ti aminte in plmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
            int poz_sus = poz_init + ridicare;
            if (poz_sus <= -2300)
                poz_sus = -2300;
            GoToPosition(robot.glisisus, poz_sus, 0.8);
        }
        GoToPosition(robot.rotatie, 100, 0.9);//de bagat in GoToPosition conditie pentru rotatie
        GoToPosition(robot.glisisus, 0, 1);
        /** poti sa bagi in gotoposition conditie sa nu treaca glisiera de -2300*/
        etaj++;

    }

    /**
     * Flag used to disable the flor increment/decrement functionality for a short period.
     */
    private boolean incrementEnabled = true;

    public void decrEtaj() {
        if (timedecr.seconds() >= 0.25) {
            incrementEnabled = true;
            timedecr.reset();
        }

        if (!incrementEnabled) {
            return;
        }

        if (gamepad1.dpad_up)
            etaj++;
        if (gamepad1.dpad_down)
            etaj--;

        incrementEnabled = false;

    }

    public void ridicare(){
        if (gamepad1.right_bumper) {
            robot.brat.setPosition(0.2);
            GoToPosition(robot.rotatie, 200, 0.5);
        }
    }

    public void GoToPosition(DcMotor motor, int ticks, double power) {
        if(motor == null)
            return;
        double curent = motor.getCurrentPosition();
        if(Math.abs(curent - ticks) <= 5)
            return;
        double minpow = 0;

        double power1 = power + 0.2;
        double power2 = power - 0.3;
        double power3 = power - 0.6;
        if (motor == robot.rotatie) {
            if (ticks <= 350 && ticks >= 40)
                minpow = 0.1;  // was 0.2
            if(ticks>=600 && ticks<= 700)
                minpow=-0.05;  /// was -0.05
            if (ticks >= 710 && ticks <= 1300)
                minpow = -0.2;  /// was -0.2
        }

        if (motor == robot.glisisus)
            minpow = -0.15;

        if (curent <= ticks) {

            while (motor.getCurrentPosition() < ticks && opModeIsActive() && !isStopRequested()) {
                if (motor == robot.rotatie) {
                    if(robot.rotatie.getCurrentPosition()>=900) {
                        power = power3;
                    }
                    else
                      if (robot.rotatie.getCurrentPosition() >= 550) { ///550
                        power = power2;
                      } else {
                        power = power1;
                      }

                }
                telemetry.addData("ticks In goToPosition",motor.getCurrentPosition());
                telemetry.addData("power In goToPosition", motor.getPower());
                telemetry.update();
                motor.setPower(power);
                Drive();

                if (gamepad2.back){
                    break;
                }

            }
            motor.setPower(minpow);

        } else {

            while (motor.getCurrentPosition() > ticks && opModeIsActive() && !isStopRequested()) {
                if (motor == robot.rotatie) {
                    if (robot.rotatie.getCurrentPosition() >= 550) { ///550
                        power = power1;
                    } else {
                        power = power2;
                    }

                }
                motor.setPower(-power);
                Drive();
                if (gamepad2.back) {
                    break;
                }

            }
            motor.setPower(minpow);
        }
    }

    public void Prinde() {
        if(robot.rotatie == null || robot.brat == null)
            return;
        if(reset)
            return;
        if (gamepad2.y) {
            GoToPosition(robot.rotatie, 40, 0.8);
            robot.brat.setPosition(0.95);
        }
    }

    public void Misca_brat() {
        if(robot.rotatie == null){
            telemetry.addLine("rotatie is null");
            return;
        }
        double pow = gamepad2.left_stick_x;
        if (pow == 0) {
            if (robot.rotatie.getCurrentPosition() <= 330 && robot.rotatie.getCurrentPosition() >= 40)
                pow = 0.2;
            if (robot.rotatie.getCurrentPosition() >= 710 && robot.rotatie.getCurrentPosition() <= 1300)
                pow = -0.2;
        } else {
            if (robot.rotatie.getCurrentPosition() <= 330 && pow > 0 && pow <= 0.4)
                pow = 0.26;
            if (robot.rotatie.getCurrentPosition() >= 710 && pow < 0 && pow >= -0.4)
                pow = -0.26;
        }

        robot.rotatie.setPower(pow);
    }

    public void Etaje(int etaj) {
        if(robot.glisisus == null || robot.rotatie == null)
            return;
        if(reset)
            return;
        if (gamepad2.x) {
            GoToPosition(robot.glisisus, initialg[etaj], 1);

            if(etaj<4)
                GoToPosition(robot.rotatie, rotatie[etaj], 0.8);
            else
                GoToPosition(robot.rotatie, rotatie[etaj], 0.5);

            while (!gamepad1.y && opModeIsActive() && !isStopRequested()) {
                Drive();
                if(etaj>=4)
                {
                    if(robot.rotatie.getCurrentPosition() > rotatie[etaj])
                     robot.rotatie.setPower(-(robot.rotatie.getCurrentPosition()-rotatie[etaj])/50);
                    else if(robot.rotatie.getCurrentPosition()< rotatie[etaj])
                    robot.rotatie.setPower((rotatie[etaj]-robot.rotatie.getCurrentPosition())/50);
                }
                if (gamepad2.back)
                    break;
            }
            robot.stopRobot();

            if(!cap)
              GoToPosition(robot.glisisus, finalg[etaj], 0.8);
            else
                GoToPosition(robot.glisisus,finalg[etaj],0.2);
            Returneaza();
        }
    }

    public void Capstone(){
        if(robot.barieraCap == null || robot.invarteCap == null || robot.lasaCap == null)
            return;
        if(gamepad1.x){
            GoToPosition(robot.rotatie, 400, 0.6);
            robot.barieraCap.setPosition(0.6);
            cap = true;
        }
        if(cap == true){
            if(robot.distanta.getDistance(DistanceUnit.MM) <= distantaCub){
                robot.bagas.setPower(0);
                robot.bagad.setPower(0);
                GoToPosition(robot.rotatie, 450, 0.6);
                robot.brat.setPosition(0.2);//lasa tata

                time.reset();
                double tm = time3.milliseconds();
                robot.invarteCap.setPosition(0.7);
                while(time3.milliseconds() - tm <= 650 && opModeIsActive() && !isStopRequested()){
                    Drive();
                }
//550
                tm = time3.milliseconds();
                robot.lasaCap.setPosition(0.1);
                while (time3.milliseconds() - tm <= 350 && opModeIsActive() && !isStopRequested()){
                    Drive();
                }

                tm = time3.milliseconds();
                robot.invarteCap.setPosition(0);
                while(time3.milliseconds() - tm <= 550 && opModeIsActive() && !isStopRequested()){
                    Drive();
                }
                cap = false;

                tm = time3.milliseconds();
                robot.barieraCap.setPosition(0.05);
                while(time3.milliseconds() - tm <= 450 && opModeIsActive() && !isStopRequested()){
                    Drive();

                }

                GoToPosition(robot.rotatie, 300, 0.6);

                robot.bagas.setPower(-1);
                robot.bagad.setPower(1);
                while(robot.distanta.getDistance(DistanceUnit.MM) > dist2 && opModeIsActive() && !isStopRequested()){
                    Drive();
                }
                robot.bagas.setPower(0);
                robot.bagad.setPower(0);
                GoToPosition(robot.rotatie, 25, 0.6);
                robot.brat.setPosition(0.95);
            }
        }
    }

    public void RidicareGlisiera()
    {
        if(robot.glisisus == null){
            telemetry.addLine("glisisus is null");
            return;
        }
        if (gamepad2.right_stick_y > 0 ) { //jos //&& robot.glisisus.getCurrentPosition() < 10
            if (n2 == 0) {
                time2.reset();
                n2 = 1;
            }
            if (pow2 == 0)
                pow2 = 0.5;
            else
                pow2 = 0.5 + time2.seconds() * 0.3;


        } else if (gamepad2.right_stick_y < 0 ) { //&& robot.atins_glisi.getVoltage() < 3.224 //&& robot.glisisus.getCurrentPosition() >= -1950
            if (n2 == 0) {
                time2.reset();
                n2 = 1;
            }
            if (pow2 == 0)
                pow2 = -0.2;
            else
                pow2 = -0.2 - time2.seconds() * 0.3;
        } else {
            pow2 = -0.1;
            n2 = 0;
        }
        robot.glisisus.setPower(pow2);

    }

    public void Resetare(){
        if(gamepad2.back){
            reset = !reset;
            if(reset){
                telemetry.addLine("Ready to reset");
                telemetry.addLine("X - ridici glisiera");
                telemetry.addLine("Y - lasi glisiera");
                telemetry.addLine("A - invarti brat inauntru");
                telemetry.addLine("B - invarti brat inafara");
                telemetry.addLine("RIGHT BUMPER - reset encoders");
                telemetry.addLine("Back - return to normal");
                telemetry.update();
            }
        }
        if(reset){
            if(gamepad2.x){
                robot.glisisus.setPower(-0.5);
            }
            else if(gamepad2.y){
                robot.glisisus.setPower(0.5);
            }
            else {
                robot.glisisus.setPower(0);
            }
            if(gamepad2.a){
                robot.rotatie.setPower(-0.5);
            }
            else if(gamepad2.b){
                robot.rotatie.setPower(0.5);
            } else{
                robot.rotatie.setPower(0);
            }
            if(gamepad2.right_bumper){
                robot.glisisus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.glisisus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.glisisus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rotatie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rotatie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rotatie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public void Aspira() {// de adaugat a doua roata
        if (robot.aspira1 == null || robot.aspira2 == null || robot.bagad == null || robot.bagas == null)
            return;
        if(reset)
            return;
        if (gamepad2.left_bumper) {//collects
            if(robot.a1.getVelocity() > -100)
                inc1 = -0.7;
            else
                inc1 = 0;

            if(robot.a2.getVelocity() < 100)
                inc2 = 0.7;
            else
                inc2 = 0;

            robot.aspira1.setPower(-0.15 + inc1);
            robot.aspira2.setPower(0.15 + inc2);
            robot.bagas.setPower(-1);
            robot.bagad.setPower(1);

        } else if (gamepad2.right_bumper) {//spits
            if(robot.a1.getVelocity() < 100)
                inc1 = 0.7;
            else
                if(robot.a2.getVelocity() > -100)
                    inc2 = -0.7;
            else
                inc1 = inc2 = 0;

            robot.aspira1.setPower(0.15 + inc1);
            robot.aspira2.setPower(-0.15 + inc2);
            robot.bagas.setPower(1);
            robot.bagad.setPower(-1);
        }
        else{
            robot.aspira1.setPower(0);
            robot.aspira2.setPower(0);
            robot.bagas.setPower(0);
            robot.bagad.setPower(0);
        }
    }

    public void Lasare() {
        if (robot.atins1 == null || robot.atins2 == null || robot.cs == null || robot.cd == null)
            return;
        if (gamepad1.a) {
            if (robot.atins1.getState())
                robot.cs.setPower(1);
            else
                robot.cs.setPower(0);
            if (robot.atins2.getState())
                robot.cd.setPower(-1);
            else
                robot.cd.setPower(0);
        } else {
            if (gamepad1.b) {
                robot.cs.setPower(-1);
                robot.cd.setPower(1);
            } else {
                robot.cs.setPower(0);
                robot.cd.setPower(0);
            }
        }

    }

    public void Prinde_TATA() {
        if(robot.brat == null) {
            telemetry.addLine("brat is empty");
            return;
        }
        if(reset)
            return;
        if (gamepad2.b)
            robot.brat.setPosition(0.2);//lasa tata
        else if (gamepad2.a)
            robot.brat.setPosition(0.85);//prinde tata
    }

    public void Ia_Placa() {
        if(robot.placas == null || robot.placad == null)
            return;
        if (gamepad2.dpad_down) {
            robot.placad.setPosition(0.5);
            robot.placas.setPosition(0.45);
        } else if (gamepad2.dpad_up) {
            robot.placad.setPosition(0.2);
            robot.placas.setPosition(0);
        }
    }

    private void resetareEtaj() {
        if (gamepad1.back)
            etaj = 0;
    }


    /**
     *  BARIERA CAP: 0.15 sus
     *               0.6 jos
     *  LASA CAP: 0.7 prinde cap
     *            0.1 lasa cap
     *  INVARTE CAP: 0 sus
     *               0.6 jos
     * */


    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addLine("Robot is initialised");
        telemetry.update();
        waitForStart();
        time.startTime();
        time2.startTime();
        timedecr.startTime();
        time3.startTime();
        while (opModeIsActive() && !isStopRequested()) {

            Drive();
            Aspira();
            Lasare();
           // RidicareGlisiera();
           // Misca_brat();
            Prinde_TATA();
            Ia_Placa();
            Prinde();
            Capstone();
            Resetare();
            Etaje(etaj);
            ridicare();
            decrEtaj();
//            telemetry.addData("atins1", robot.atins1.getState());
//            telemetry.addData("distanta", robot.distanta.getDistance(MM));
//            telemetry.addData("placa", robot.placa.getDistance(MM));
//            telemetry.addData("atins2", robot.atins2.getState());
//            telemetry.addData("rotatie", robot.rotatie.getCurrentPosition());
//            telemetry.addData("etaj",etaj);
//            telemetry.update();

            resetareEtaj();

         /*  telemetry.addData("stare1", robot.atins1.getState());
            telemetry.addData("stare2", robot.atins2.getState());
            telemetry.addData("count", robot.glisisus.getCurrentPosition());
            telemetry.addData("count_rotatie", robot.rotatie.getCurrentPosition());
            telemetry.addData("power rotatie", robot.rotatie.getPower());
            telemetry.addData("glisiera", robot.glisisus.getCurrentPosition());
           telemetry.addData("servo position", robot.brat.getPosition());*/
            //telemetry.addData("etaj", etaj);

         /*   telemetry.addData("left_x", gamepad1.left_stick_x);
            telemetry.addData("left_y", gamepad1.left_stick_y);
            telemetry.addData("right_x", gamepad1.right_stick_x);
            telemetry.addData("right_y", gamepad1.right_stick_y);
            telemetry.addData("atan2",Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI / 4);
            telemetry.addData("toDegrees", Math.toDegrees(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x)));
            telemetry.addData("aspira1", robot.aspira1.getCurrentPosition());
            telemetry.addData("aspira2", robot.aspira2.getCurrentPosition());*/
//            telemetry.addData("distanta", robot.placa.getDistance(MM));
//            telemetry.addData("pow1", robot.aspira1.getPower());
//            telemetry.addData("pow2", robot.aspira2.getPower());
//            telemetry.addData("a1 velocity", robot.a1.getVelocity());
//            telemetry.addData("a2 velocity", robot.a2.getVelocity());
//
//
//            telemetry.addData("X:", robot.imu.getVelocity().xVeloc);
//            telemetry.addData("Y:", robot.imu.getVelocity().yVeloc);
//            telemetry.addData("Z:", robot.imu.getVelocity().zVeloc);
//
//
//            telemetry.addData("distanta mm", robot.distanta.getDistance(DistanceUnit.MM));
//            telemetry.addData("distanta cm", robot.distanta.getDistance(DistanceUnit.CM));
//            if(robot.placa != null){
//                telemetry.addData("placa mm", robot.placa.getDistance(DistanceUnit.MM));
//                telemetry.addData("placa cm", robot.placa.getDistance(DistanceUnit.CM));
//            }
//            else telemetry.addLine("placa is null");

           // telemetry.update();
        }

    }

}