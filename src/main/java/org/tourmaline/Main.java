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
import org.tourmaline.RigidBody.RigidBody;


import java.util.ArrayList;

import static org.tourmaline.PlanePhysics.Airfoil.Airfoil.arrayToList;

public class Main {
    public static void main(String[] args) {
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

        float simulationTime = 100.0f; // Total simulation time in seconds
        float timeStep = 0.1f; // Time step for simulation

        System.out.println("Starting plane simulation...");

        // Table Header
        String tableHeader = String.format("+------------+-------------------------+-------------------------+-------------------------+-------------------------+-------------------------+%n" +
                        "| %-10s | %-23s | %-23s | %-23s | %-23s | %-23s |%n" +
                        "+------------+-------------------------+-------------------------+-------------------------+-------------------------+-------------------------+%n",
                "Time (s)", "Position", "Velocity", "Orientation", "Angular Velocity", "Angular Acceleration");

        System.out.println(tableHeader);

        // Run the simulation
        plane.setControlInput(0.0f,0,0.0f);
        plane.getVelocity().x = 160;
        plane.setEnableGravity(true);
//        float max_av = 45.0f;  // deg/s
//        float target_av = max_av * joystick.elevator;
//        float current_av = glm::degrees(player.airplane.angular_velocity.z);
//        player.airplane.joystick.z = pitch_control_pid.calculate(current_av, target_av, dt);

        PID pid = new PID(1.0f, 0.0f, 0.0f);

        for (float t = 0; t < simulationTime; t += timeStep){

            float elevator = 0;
            float max_av = 45.0f;
            float target_av = max_av * elevator;
            float current_av = Math.toDegrees(plane.getAngularVelocity().z);

            elevator = pid.calculate(current_av, target_av, timeStep);
            
            plane.setControlInput(0.0f, elevator,0.0f);
            plane.update(timeStep);

            // Get the necessary data
            String positionStr = plane.getPosition().toString();
            String velocityStr = plane.getVelocity().toString();
            String orientationStr = String.valueOf(plane.getOrientation().getEulerAnglesXYZ(new Vector3f())); // Assuming `getOrientation()` returns a string representation of the orientation
            String angularVelocityStr = plane.getAngularVelocity().toString(); // Assuming `getAngularVelocity()` returns a string representation
            String angularAccelerationStr = plane.getAcceleration().toString(); // Assuming `getAngularAcceleration()` returns a string


            System.out.println(orientationStr);
            // Print the current values in the table format
//            System.out.printf("| %-10.2f | %-23s | %-23s | %-23s | %-23s | %-23s |%n",
//                    t, positionStr, velocityStr, orientationStr, angularVelocityStr, angularAccelerationStr);
        }


        System.out.println("+------------+-------------------------+-------------------------+-------------------------+-------------------------+-------------------------+");



    }

    static  Vector3f toDeg(Quaternionf quat){
        Vector3f vec = quat.getEulerAnglesXYZ(new Vector3f());

        vec.x = Math.toDegrees(vec.x);
        vec.y = Math.toDegrees(vec.y);
        vec.z = Math.toDegrees(vec.z);

        return vec;

    }


}

