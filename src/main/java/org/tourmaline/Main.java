package org.tourmaline;

import org.joml.Math;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.tourmaline.PlanePhysics.Airfoil.Airfoil;
import org.tourmaline.PlanePhysics.Airfoil.Constants;
import org.tourmaline.PlanePhysics.Engine;
import org.tourmaline.PlanePhysics.PID;
import org.tourmaline.PlanePhysics.Plane;
import org.tourmaline.PlanePhysics.Wing;
import org.tourmaline.Processing.PhysicsProcessor;
import org.tourmaline.RigidBody.RigidBody;


import java.util.ArrayList;

import static org.tourmaline.PlanePhysics.Airfoil.Airfoil.arrayToList;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        Matrix3f inertia = new Matrix3f(
                46311.668f, -660.000f, -0.000f,
                -660.000f,188713.797f,-0.000f,
                -0.000f, -0.000f,147367.125f);

        Engine jetEngine = new Engine(13000);
        ArrayList<Wing> wings = new ArrayList<>();
        float wing_offset = -1.0f;
        float tail_offset = -6.6f;
        Airfoil airfoil0012 = new Airfoil(arrayToList(Constants.NACA_0012));

        Airfoil airfoil2412 = new Airfoil(arrayToList(Constants.NACA_2412));


        wings.add(new Wing(new Vector3f(wing_offset,0, -2.7f), 6.96f, 2.50f,
                airfoil2412, new Vector3f(0,1,0), 0.2f));

        wings.add(new Wing(new Vector3f(wing_offset,0, 2.7f), 6.96f, 2.50f,
                airfoil2412, new Vector3f(0,1,0), 0.2f));

        wings.add(new Wing(new Vector3f(tail_offset,-0.1f, 0.0f), 6.54f, 2.70f,
                airfoil0012, new Vector3f(0,1,0), 1f));

        wings.add(new Wing(new Vector3f(tail_offset,0.0f, 0.0f), 5.31f, 3.1f,
                airfoil0012, new Vector3f(0,0,1), 0.15f));

        Plane plane = new Plane(inertia, new Vector3f(0,8000,0), 9000, jetEngine, wings);


        RigidBody rb1 = new RigidBody(inertia, new Vector3f(1000), 500);
        rb1.setEnableGravity(true);


        RigidBody rb2 = new RigidBody(inertia, new Vector3f(800), 300);
        rb2.setEnableGravity(true);

        rb2.applyForce(new Vector3f(1000,0,0));

        PhysicsProcessor physicsProcessor = new PhysicsProcessor(new ArrayList<>());
        physicsProcessor.addRigidBody(rb1);     physicsProcessor.addRigidBody(rb2);

        physicsProcessor.start();
        physicsProcessor.join();


    }

    static  Vector3f toDeg(Quaternionf quat){
        Vector3f vec = quat.getEulerAnglesXYZ(new Vector3f());

        vec.x = Math.toDegrees(vec.x);
        vec.y = Math.toDegrees(vec.y);
        vec.z = Math.toDegrees(vec.z);

        return vec;

    }


}

