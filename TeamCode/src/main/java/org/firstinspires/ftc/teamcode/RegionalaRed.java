package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.TickMeter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

@Autonomous(name = "Regionala Red", group = "Linear OpMode")


public class RegionalaRed extends LinearOpMode {
    OpenCvCamera cam1;

    Hardware robot = new Hardware();

    public double asta;

    final double TICKSURI_PER_CM = 27.3696374577;

    private double inc1;
    private double inc2;
    private final double vit_max = 1;
    private final double lat_robot = 35;
    private final int ridicare = -450;

    public enum Direction{
        FRONT, BACK, LEFT, RIGHT;
    }

    double unghi(double u) {
        if (u < 0)
            u += 360;
        else if (u > 360)
            u -= 360;
        return u;
    }

    public void rotate(double speed){
        ///rotire in sens antitrigonometric in cazul puterii pozitive
        robot.front_left.setPower(speed);
        robot.front_right.setPower(-speed);
        robot.back_left.setPower(speed);
        robot.back_right.setPower(-speed);
    }

    public void Arc(Direction dir1, Direction dir2, double r, double angle, DcMotor glisisus, double ticks1, double pow1, DcMotor rotatie, double ticks2, double pow2) {
        if((dir1 == Direction.FRONT && dir2 == Direction.RIGHT)||( dir1 == Direction.BACK && dir2 == Direction.LEFT)) {
            angle = -angle;
        }
        double error = 10;
        double current = unghi(robot.imu.getAngularOrientation().firstAngle);
        double target = unghi(current + angle);

        double remaining = Math.abs(angle);
        double ct = r / lat_robot;
        double in =-1.2* vit_max * r / (r + lat_robot);// ct * ex * r / (r + lat_robot)
        double ex = vit_max;

        double fl,fr,bl,br;
        if(dir2 == Direction.RIGHT){
            fl = bl = ex;
            fr = in; br = 0;
        } else {
            fl = in; bl = 0;
            fr = br = ex;
        }
        if(dir1 == Direction.BACK){
            fl = -fl; fr = -fr;
            bl = -bl; br = -br;
        }

        double curent1=0, minpow1= -0.15;
        double curent2=0, minpow2=0, pr1 = pow2 + 0.2, pr2 = pow2 - 0.3;
        boolean glis = false;
        boolean rot = false;
        boolean stop = false;
        boolean brat = false;
        boolean tm = false;
        boolean glisfinal = false;
        double poz = -5500;

        ElapsedTime time = new ElapsedTime();
        if(glisisus != null){
            curent1 = glisisus.getCurrentPosition();
            glis = true;
        }

        if(rotatie != null){
            curent2 = rotatie.getCurrentPosition();
            if(ticks2 <= 350 && ticks2 >= 40)
                minpow2 = 0.2;
            else if(ticks2 >= 710 && ticks2 <= 1300)
                minpow2 = -0.2;
        }

        while (opModeIsActive() && !isStopRequested() && (remaining > error || glis || rot || glisfinal || !stop)) {
            current = unghi(robot.imu.getAngularOrientation().firstAngle);
            if (angle > 0) {
                if (target < current && current - target > 180) {
                    remaining = target - current + 360;
                } else {
                    remaining = target - current;
                }

            } else {
                if (target > current && target - current > 180) {
                    remaining = current - target + 360;
                } else {
                    remaining = current - target;
                }
            }
            telemetry.addData("Remaining", remaining);
            telemetry.addData("current", current);
            telemetry.addData("target", target);
            telemetry.update();
            if(remaining < error && !stop){
                robot.stopRobot();
                stop = true;
            }
            else if(!stop){
                robot.front_left.setPower(fl);
                robot.back_left.setPower(bl);
                robot.front_right.setPower(fr);
                robot.back_right.setPower(br);
            }
            if(glisisus != null ){
                if(glis) {
                    if ((curent1 <= ticks1 && glisisus.getCurrentPosition() >= ticks1) || (curent1 >= ticks1 && glisisus.getCurrentPosition() <= ticks1)) {
                        glisisus.setPower(minpow1);
                        glis = false;
                    } else {
                        if (curent1 <= ticks1) {
                            glisisus.setPower(pow1);
                        } else {
                            glisisus.setPower(-pow1);
                        }
                    }
                }
                else if(!rot){
                    if(!brat) {
                        if(!tm){
                            time.reset();
                            tm = true;
                            robot.brat.setPosition(0.2);
                        }
                        else{
                            if(time.milliseconds() > 200){
                                brat = true;
                            }
                        }
                    }
                    else{
                        if(poz == -5500){
                            curent1 = glisisus.getCurrentPosition();
                            poz = Math.max(-2300, curent1 + ridicare);
                            pow1 = 0.8;
                        }
                        else{
                            if ((curent1 <= poz && glisisus.getCurrentPosition() >= poz) || (curent1 >= poz && glisisus.getCurrentPosition() <= poz)) {
                                glisisus.setPower(minpow1);
                                glis = false;
                                curent1 = glisisus.getCurrentPosition();
                                rot = true;
                            } else {
                                if (curent1 <= ticks1) {
                                    glisisus.setPower(pow1);
                                } else {
                                    glisisus.setPower(-pow1);
                                }
                            }
                        }
                    }
                }
            }

            if(rotatie != null && rot){
                if ((curent2 <=ticks2 && rotatie.getCurrentPosition() >= ticks2)||(curent2 >= ticks2 && rotatie.getCurrentPosition() <= ticks2)) {
                    rotatie.setPower(minpow2);
                    rot = false;
                    glisfinal = true;
                } else {
                    if (curent2 <= ticks2) {
                        if (robot.rotatie.getCurrentPosition() >= 550) {
                            pow2 = pr2;
                        } else {
                            pow2 = pr1;
                        }
                        rotatie.setPower(pow2);
                    } else {
                        if (rotatie.getCurrentPosition() >= 550) {
                            pow2 = pr1;
                        }
                        rotatie.setPower(-pow2);
                    }
                }

            }

            if(glisfinal) {
                if ((curent1 <= 0 && glisisus.getCurrentPosition() >= 0) || (curent1 >= 0 && glisisus.getCurrentPosition() <= 0)) {
                    glisisus.setPower(0);
                    glisfinal = false;
                } else {
                    if (curent1 <= 0) {
                        glisisus.setPower(pow1);
                    } else {
                        glisisus.setPower(-pow1);
                    }
                }
            }

        }

        Redreseaza(target, false);
        robot.stopRobot();
    }

    public void Redreseaza(double target,boolean brate) {
        double MAXSPEED=0.25;
        double current = unghi(robot.imu.getAngularOrientation().firstAngle);
        double angle = target - current;

        double remaining = Math.abs(angle);

        double speed = MAXSPEED ;

        while (remaining > 1 && opModeIsActive() && !isStopRequested() ) {
            if(brate){
                if(!robot.atins1.getState())
                    robot.cs.setPower(0);
                if(!robot.atins2.getState())
                    robot.cd.setPower(0);
            }
            current = unghi(robot.imu.getAngularOrientation().firstAngle);

            if(Math.abs(robot.imu.getAngularVelocity().xRotationRate)<0.4)
                speed+=0.03;

            if (angle > 0) {
                if (target < current && current - target > 180) {
                    remaining = target - current + 360;
                } else {
                    remaining = target - current;
                }
                rotate(-speed);
            }
            else {
                if (target > current && target - current > 180) {
                    remaining = current - target + 360;

                } else {
                    remaining = current - target;
                }

                rotate(speed);
            }
//            telemetry.addData("RRRremaining", remaining);
//            telemetry.update();
        }
        robot.stopRobot();
    }

    public void gyroTurnV5(double angle, boolean brate, double power, double ticks, DcMotor motor) {
        double error = 2;
        double MAXSPEED = 1;
        double speed = MAXSPEED ;

        double curent=0, minpow=0,power1=0,power2=0;

        if(motor!=null)
        {
            curent = motor.getCurrentPosition();
            minpow = 0;

            power1 = power + 0.2;
            power2 = power - 0.3;
            if (motor == robot.rotatie) {
                if (ticks <= 350 && ticks >= 40)
                    minpow = 0.2;
                if (ticks >= 710 && ticks <= 1300)
                    minpow = -0.2;
            }

            if (motor == robot.glisisus)
                minpow = -0.15;
        }

        double current = unghi(robot.imu.getAngularOrientation().firstAngle);
        double target = unghi(current + angle);
        telemetry.addLine("current" + current);
        telemetry.addLine("target" + target);
        telemetry.update();

        double remaining = Math.abs(angle);
        double total = remaining / 2;
        if(angle<30)
            MAXSPEED=0.45;
        double viteza=100;

        while ((opModeIsActive() && !isStopRequested() )&& (remaining > error &&  viteza>0.07  )
                || (motor!=null && ((motor.getCurrentPosition() < ticks && curent <= ticks) ||(motor.getCurrentPosition() > ticks && curent >= ticks)))) {
            if(brate)
            {
                if(!robot.atins1.getState())
                    robot.cs.setPower(0);
                if(!robot.atins2.getState())
                    robot.cd.setPower(0);
            }

            if(motor!=null)
            {
                if ((curent <=ticks && motor.getCurrentPosition() >= ticks)||(curent >=ticks && motor.getCurrentPosition() <= ticks))
                {motor.setPower(minpow);
                }
                else {
                    if (curent <= ticks) {

                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power2;
                            } else {
                                power = power1;
                            }

//                            telemetry.addData("ticks In goToPosition", motor.getCurrentPosition());
//                            telemetry.addData("power In goToPosition", motor.getPower());
//                            telemetry.update();
                            motor.setPower(power);

                        }

                    } else {

                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power1;
                            }
                        }
                        motor.setPower(-power);
                    }
                }
            }

            if(remaining>total)
                viteza=100;
            else
                viteza=Math.abs(robot.imu.getAngularVelocity().xRotationRate);

            telemetry.addData("viteza", robot.imu.getAngularVelocity().xRotationRate);
            current = unghi(robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("target", target);
            if (angle > 0) {
                if (target < current && current - target > 180) {
                    telemetry.addData("1", current);
                    remaining = target - current + 360;
                } else {
                    telemetry.addData("2", current);
                    remaining = target - current;
                }
                if (remaining <= total) {
                    speed = MAXSPEED - ((total - remaining) * ((MAXSPEED - (0)) / total));
                }
                if(speed<0.35)
                    speed=0;
                rotate(-speed);
            } else {
                if (target > current && target - current > 180) {
                    remaining = current - target + 360;

                } else {
                    remaining = current - target;
                }
                if (remaining <= total) {
                    speed = MAXSPEED - ((total - remaining) * ((MAXSPEED - (0)) / total));
                }

                if(speed<0.35)
                    speed=0;
                rotate(speed);
            }
            telemetry.addData("glisisus", robot.glisisus.getCurrentPosition());
            telemetry.addData("remaining", remaining);
            telemetry.update();
        }
        asta = remaining;
        Redreseaza(target,brate);


        robot.stopRobot();
    }

    public void mersTicksuri(double distance, Direction dir, double pow, DcMotor motor, double ticks,
                             double power, boolean brate, boolean colectare, boolean tata){
        double curent=0, minpow=0,power1=0,power2=0;

        if(motor!=null)
        {
            curent = motor.getCurrentPosition();
            minpow = 0;

            power1 = power + 0.2;
            power2 = power - 0.3;
            if (motor == robot.rotatie) {
                if (ticks <= 350 && ticks >= 40)
                    minpow = 0.2;
                if (ticks >= 710 && ticks <= 1300)
                    minpow = -0.2;
            }

            if (motor == robot.glisisus)
                minpow = -0.15;
        }
        double fl,fr,br,bl;
        double speed = -1 * pow;
        fl = fr = br = bl = -1;
        if(dir == Direction.BACK)
            fl = fr = br = bl = 1;
        else if(dir == Direction.LEFT)
            fl = br = 1;
        else if(dir == Direction.RIGHT)
            fr = bl = 1;
        double initial = robot.back_left.getCurrentPosition();
        double necesar = distance * TICKSURI_PER_CM;
        double rem = 15 * TICKSURI_PER_CM;    /// decelerez pe ultimii 15 cm
        robot.front_left.setPower(speed * fl);
        robot.front_right.setPower(speed * fr);
        robot.back_right.setPower(speed * br);
        robot.back_left.setPower(speed * bl);
        boolean ver  = true;
        boolean b1 = false, b2= false;
        boolean mot = true;

        while((Math.abs(robot.back_left.getCurrentPosition() - initial)<necesar && (Math.abs(robot.m.getVelocity()) > 60 || ver))
                || (brate && (!b1 || !b2)) ||(motor!=null && mot)
                && !isStopRequested() && opModeIsActive() ){
            telemetry.addData("in MersTicks ", 1);
            telemetry.addData("b1", b1);
            telemetry.addData("B2", b2);
            telemetry.addData("rotatie", robot.rotatie.getCurrentPosition());
            telemetry.update();
            if(colectare)
            {
                if(robot.a1.getVelocity() > -100)
                    inc1 = -0.7;
                else
                    inc1 = 0;

                if(robot.a2.getVelocity() < 100)
                    inc2 = 0.7;
                else
                    inc2 = 0;

                robot.aspira1.setPower(-0.3 + inc1);
                robot.aspira2.setPower(0.3 + inc2);
                robot.bagas.setPower(-1);
                robot.bagad.setPower(1);

                telemetry.addData("a1 velocity", robot.a1.getVelocity());
                telemetry.addData("a2 velocity", robot.a2.getVelocity());
                telemetry.addData("a1 power", robot.aspira1.getPower());
                telemetry.addData("a2 power", robot.aspira2.getPower());
                telemetry.update();
            }
            if(brate){
                if (robot.atins1.getState()){
                    robot.cs.setPower(1);
                }
                else{
                    b1 = true;
                    robot.cs.setPower(0);
                }
                if (robot.atins2.getState())
                    robot.cd.setPower(-1);
                else{
                    b2 = true;
                    robot.cd.setPower(0);
                }
            }
            if(motor!=null) {

                if ((curent <=ticks && motor.getCurrentPosition() >= ticks)||(curent >=ticks && motor.getCurrentPosition() <= ticks)) {
                    motor.setPower(minpow);
                    mot = false;
                }
                else {
                    if (curent <= ticks) {
                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power2;
                            } else {
                                power = power1;
                            }
                        }
                        motor.setPower(power);

                    } else {
                        if (motor == robot.rotatie && robot.rotatie.getCurrentPosition() >= 550) {
                            power = power1;
                        }
                        motor.setPower(-power);
                    }
                }
            }
            double d = necesar - Math.abs(robot.back_left.getCurrentPosition() - initial);
            if(d < rem){
                ver=false;
                double v = -1 * pow * d / rem;
                if(v>-0.47 * pow)
                    v=0;
                robot.front_left.setPower(v * fl);
                robot.front_right.setPower(v * fr);
                robot.back_left.setPower(v * bl);
                robot.back_right.setPower(v * br);
            }
        }


        if(colectare && !isStopRequested() && opModeIsActive()) {
            while(robot.distanta.getDistance(MM)>60 && opModeIsActive() && !isStopRequested()){
                if(robot.a1.getVelocity() > -100)
                    inc1 = -0.7;
                else
                    inc1 = 0;

                if(robot.a2.getVelocity() < 100)
                    inc2 = 0.7;
                else
                    inc2 = 0;

                if(robot.distanta.getDistance(MM) <270){
                    inc1 = 1;
                    inc2 = -1;
                }
                robot.aspira1.setPower(-0.3 + inc1);
                robot.aspira2.setPower(0.3 + inc2);

                robot.bagas.setPower(-1);
                robot.bagad.setPower(1);
            }

            robot.aspira1.setPower(0);
            robot.aspira2.setPower(0);
            robot.bagas.setPower(0);
            robot.bagad.setPower(0);
        }

        if(tata && !isStopRequested() && opModeIsActive()) {
           if(robot.brat.getPosition() == 0.9)
            robot.brat.setPosition(0.2);
           else
               robot.brat.setPosition(0.9);
        }
    }

    private void mersDistanta(double pow, DcMotor motor, double ticks, double power){
        double curent=0, minpow=0,power1=0,power2=0;

        if(motor!=null) {
            curent = motor.getCurrentPosition();
            minpow = 0;

            power1 = power + 0.2;
            power2 = power - 0.3;
            if (motor == robot.rotatie) {
                if (ticks <= 350 && ticks >= 40)
                    minpow = 0.2;
                if (ticks >= 710 && ticks <= 1300)
                    minpow = -0.2;
            }

            if (motor == robot.glisisus)
                minpow = -0.15;
        }
        double speed = -pow;
        double initial = robot.placa.getDistance(MM);
        double dinit = robot.back_left.getCurrentPosition();
        // double necesar = initial - dist;
        double rem = initial/2;    /// decelerez pe ultimii 15 cm
        double limit = 20 * TICKSURI_PER_CM;
        robot.front_left.setPower(speed);
        robot.front_right.setPower(speed);
        robot.back_right.setPower(speed);
        robot.back_left.setPower(speed);
        boolean ver = true;

        while((ver ||(motor!=null && ((motor.getCurrentPosition() < ticks && curent <= ticks) ||(motor.getCurrentPosition() > ticks && curent >= ticks))))
                && !isStopRequested() && opModeIsActive()){
            if(motor!=null) {
                if ((curent <=ticks && motor.getCurrentPosition() >= ticks)||(curent >=ticks && motor.getCurrentPosition() <= ticks))
                {motor.setPower(minpow);
                }
                else {
                    if (curent <= ticks) {

                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power2;
                            } else {
                                power = power1;
                            }

                            motor.setPower(power);
                        }
                    } else {
                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power1;
                            }
                        }
                        motor.setPower(-power);
                    }
                }

            }

            if((robot.placa.getDistance(MM) < 40 || Math.abs(robot.back_left.getCurrentPosition() - dinit) >= limit)&& ver){
                robot.stopRobot();
                ver = false;
            }

        }
        robot.stopRobot();

    }

    public void GoToPosition(DcMotor motor, int ticks, double power) {
        if(motor == null)
            return;
        double curent = motor.getCurrentPosition();
        double minpow = 0;

        double power1 = power + 0.2;
        double power2 = power - 0.3;
        if (motor == robot.rotatie) {
            if (ticks <= 350 && ticks >= 40)
                minpow = 0.2;
            if (ticks >= 710 && ticks <= 1300)
                minpow = -0.2;
        }

        if (motor == robot.glisisus)
            minpow = -0.15;

        if (curent <= ticks) {

            while (motor.getCurrentPosition() < ticks && opModeIsActive() && !isStopRequested()) {
                if (motor == robot.rotatie) {
                    if (robot.rotatie.getCurrentPosition() >= 550) {
                        power = power2;
                    } else {
                        power = power1;
                    }

                }
//                telemetry.addData("ticks In goToPosition",motor.getCurrentPosition());
//                telemetry.addData("power In goToPosition", motor.getPower());
//                telemetry.update();
                motor.setPower(power);



            }
            motor.setPower(minpow);

        } else {

            while (motor.getCurrentPosition() > ticks && opModeIsActive() && !isStopRequested() ) {
                if (motor == robot.rotatie) {
                    if (robot.rotatie.getCurrentPosition() >= 550) {
                        power = power1;
                    }
                }
                motor.setPower(-power);
            }
            motor.setPower(minpow);
        }
    }

    private void Autonomie_Stanga(){
        gyroTurnV5(10-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);
        mersTicksuri(70, Direction.FRONT,0.3,null,0,0,false,true,false);
        mersTicksuri(42, Direction.BACK,1,robot.rotatie,30,0.6,false,false,true);

        gyroTurnV5(90-robot.imu.getAngularOrientation().firstAngle, false,0,0,null);


        mersTicksuri(215, Direction.BACK,1,null,0,0,false,false,false);
        gyroTurnV5(180-robot.imu.getAngularOrientation().firstAngle,false,1,-900,robot.glisisus);

        robot.placad.setPosition(0.35);
        robot.placas.setPosition(0.2);
        mersDistanta(0.2,robot.rotatie,1500,0.6);

        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);
        sleep(300);
        //mersTicksuri(5, Direction.FRONT,1,null,0,0,false,false,false);


        Arc(Direction.FRONT, Direction.RIGHT, 10, 90, robot.glisisus, -600, 0.8, robot.rotatie, 100, 0.7);// de fact simutan motor

        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);
        Redreseaza(90, false);

        mersTicksuri(180 , Direction.FRONT,1,robot.glisisus,0,0.7,false,false,false);
        gyroTurnV5(60-robot.imu.getAngularOrientation().firstAngle,false,0.4,200,robot.rotatie);
        // Redreseaza(300, false);
        mersTicksuri(55, Direction.FRONT, 0.3, null,0,0,false,true,false);//ia stone

        mersTicksuri(35, Direction.BACK,1,robot.rotatie,0,0.5,false,false,true); // se intoarce
        gyroTurnV5(90-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);

        mersTicksuri(160, Direction.BACK,1,null,0,0,false,false,false);
        mersTicksuri(20, Direction.BACK,1,robot.glisisus,-1150,1,false,false,false);
        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);


        mersTicksuri(20, Direction.BACK,1,robot.rotatie,1500,0.8,false,false,false);
        mersTicksuri(20 , Direction.BACK,1,robot.glisisus,-850,0.7,false,false,false);

        robot.brat.setPosition(0.2);
        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);

        GoToPosition(robot.glisisus,-1200,1);
        GoToPosition(robot.rotatie, 50, 0.6);

        mersTicksuri(90, Direction.FRONT,1,robot.glisisus,0,0.7,false,false,false);

    }

    private void Autonomie_Dreapta(){
        gyroTurnV5(-22-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);
        mersTicksuri(70, Direction.FRONT,0.3,null,0,0,false,true,false);
        mersTicksuri(40, Direction.BACK,1,robot.rotatie,30,0.6,false,false,true);

        gyroTurnV5(-90-robot.imu.getAngularOrientation().firstAngle, false,0,0,null);


        mersTicksuri(200, Direction.FRONT,1,null,0,0,false,false,false);
        gyroTurnV5(-180-robot.imu.getAngularOrientation().firstAngle,false,1,-900,robot.glisisus);

        robot.placad.setPosition(0.35);
        robot.placas.setPosition(0.2);
        mersDistanta(0.2,robot.rotatie,1500,0.6);

        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);
        sleep(300);
        mersTicksuri(5, Direction.FRONT,1,null,0,0,false,false,false);

        Arc(Direction.FRONT, Direction.RIGHT, 10, 90, robot.glisisus, -600, 0.7, robot.rotatie, 100, 0.7);// de fact simutan motor

        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);

        Redreseaza(90, false);

        mersTicksuri(152 , Direction.FRONT,1,robot.glisisus,0,0.7,false,false,false);
        gyroTurnV5(60-robot.imu.getAngularOrientation().firstAngle,false,0.4,200,robot.rotatie);
        // Redreseaza(300, false);
        mersTicksuri(48, Direction.FRONT, 0.3, null,0,0,false,true,false);//ia stone

        mersTicksuri(38, Direction.BACK,1,robot.rotatie,0,0.5,false,false,true); // se intoarce
        gyroTurnV5(90-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);

        mersTicksuri(160, Direction.BACK,1,null,0,0,false,false,false);

        mersTicksuri(20, Direction.BACK,1,robot.glisisus,-1150,1,false,false,false);
        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);


        mersTicksuri(20, Direction.BACK,1,robot.rotatie,1500,0.8,false,false,false);
        mersTicksuri(20 , Direction.BACK,1,robot.glisisus,-850,0.7,false,false,false);

        robot.brat.setPosition(0.2);
        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);

        GoToPosition(robot.glisisus,-1200,1);
        GoToPosition(robot.rotatie, 50, 0.6);


        mersTicksuri(90, Direction.FRONT,1,robot.glisisus,0,0.6,false,false,false);

    }

    private void Autonomie_Mijloc(){
        gyroTurnV5(-10,false,0,0,null);
        mersTicksuri(60, Direction.FRONT,0.3,null,0,0,false,true,false);
        mersTicksuri(28, Direction.BACK,1,robot.rotatie,30,0.5,false,false,true);

        gyroTurnV5(-90-robot.imu.getAngularOrientation().firstAngle, false,0,0,null);


        mersTicksuri(205, Direction.FRONT,1,null,0,0,false,false,false);
        gyroTurnV5(-180-robot.imu.getAngularOrientation().firstAngle,false,1,-900,robot.glisisus);

        mersDistanta(0.2,robot.rotatie,1500,0.6);

        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);
        sleep(300);
        mersTicksuri(7, Direction.FRONT,1,null,0,0,false,false,false);

        Arc(Direction.FRONT, Direction.RIGHT, 10, 90, robot.glisisus, -600, 0.7, robot.rotatie, 100, 0.7);// de fact simutan motor

        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);

        Redreseaza(90, false);

        mersTicksuri(168 , Direction.FRONT,1,null,0,0,false,false,false);
        gyroTurnV5(55-robot.imu.getAngularOrientation().firstAngle,false,0.6,200,robot.rotatie);
        // Redreseaza(300, false);
        mersTicksuri(40, Direction.FRONT, 0.3, null,0,0,false,true,false);//ia stone

        mersTicksuri(30, Direction.BACK,1,robot.rotatie,0,0.5,false,false,true); // se intoarce
        gyroTurnV5(90-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);

        mersTicksuri(165, Direction.BACK,1,null,0,0,false,false,false);

        mersTicksuri(20, Direction.BACK,1,robot.glisisus,-1150,1,false,false,false);
        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);


        mersTicksuri(20, Direction.BACK,1,robot.rotatie,1500,0.8,false,false,false);
        mersTicksuri(20 , Direction.BACK,1,robot.glisisus,-850,0.7,false,false,false);

        robot.brat.setPosition(0.2);
        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);

        GoToPosition(robot.glisisus,-1200,1);
        GoToPosition(robot.rotatie, 50, 0.6);


        mersTicksuri(90, Direction.FRONT,1,robot.glisisus,0,0.6,false,false,false);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam1 = OpenCvCameraFactory.getInstance().createWebcam(robot.web, cameraMonitorViewId);

        for(int i=1; i<=100; i++)
            cam1.openCameraDevice();

        cam1.setPipeline(new proces2());
        cam1.startStreaming(640,480);

        telemetry.addLine("Robot is initialised");
        telemetry.update();
        waitForStart();
        String motori = Test2.pos;
        telemetry.addData("Pozitie", motori);
        telemetry.update();
        //cam1.stopStreaming();

        mersTicksuri(35, Direction.FRONT,1, robot.rotatie, 200, 0.4, true, false, false);
        if(motori.equals("left")) {
            Autonomie_Stanga();
        } else if(motori.equals("right")) {
            Autonomie_Dreapta();
        } else if(motori.equals("middle")) {
            Autonomie_Mijloc();
        }

    }
}