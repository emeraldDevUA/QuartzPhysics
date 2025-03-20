package org.tourmaline;

import org.joml.Math;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.tourmaline.Collision.CollisionPrimitive;
import org.tourmaline.PlanePhysics.Airfoil.Airfoil;
import org.tourmaline.PlanePhysics.Airfoil.Constants;
import org.tourmaline.PlanePhysics.Engine;
import org.tourmaline.PlanePhysics.Tuple;
import org.tourmaline.Processing.PhysicsProcessor;
import org.tourmaline.RigidBody.InertiaCalculator;
import org.tourmaline.RigidBody.InertiaPrimitives.Cuboid;
import org.tourmaline.RigidBody.InertiaPrimitives.InertiaPrimitive;
import org.tourmaline.RigidBody.RigidBody;
import org.tourmaline.Units.Length.LengthUnits;
import org.tourmaline.Units.Speed.SpeedUnitConverter;
import org.tourmaline.Units.Time.TimeUnits;


import java.util.ArrayList;
import java.util.List;

import static org.tourmaline.PlanePhysics.Airfoil.Airfoil.arrayToList;

import static org.tourmaline.Units.Length.LengthUnits.*;
import static org.tourmaline.Units.Time.TimeUnits.*;

public class Main {
    public static void main(String[] args) throws InterruptedException {
            Matrix3f inertia = new Matrix3f(
                    46311.668f, -660.000f, -0.000f,
                    -660.000f,188713.797f,-0.000f,
                    -0.000f, -0.000f,147367.125f);
    //
    //        Engine jetEngine = new Engine(13000);
    //        ArrayList<Wing> wings = new ArrayList<>();
            float wing_offset = -1.0f;
            float tail_offset = -6.6f;


        SpeedUnitConverter speedUnitConverter = new SpeedUnitConverter();
        float mps = speedUnitConverter.convert(1,
                new Tuple<>(KILOMETER, HOUR),
                new Tuple<>(METER, SECOND));

        System.out.println(mps);

            RigidBody rb1 = new RigidBody(inertia, new Vector3f(1000), 500);
            rb1.setEnableGravity(true);
            RigidBody rb2 = new RigidBody(inertia, new Vector3f(800), 300);
            rb2.setEnableGravity(true);

        float plane_mass = 10000f;
        List<InertiaPrimitive> list = new ArrayList<>();

        list.add(new Cuboid(new Vector3f(wing_offset,0, 2.7f),
                new Vector3f(6.96f, 0.10f, 3.50f), plane_mass*1/4));
        list.add(new Cuboid(new Vector3f(wing_offset,0, -2.7f),
                new Vector3f(6.96f, 0.10f, 3.50f), plane_mass*1/4));

        list.add(new Cuboid(new Vector3f(tail_offset,-0.1f, 0f),
                new Vector3f(6.54f, 0.10f, 2.70f), plane_mass*1/10));

        list.add(new Cuboid(new Vector3f(tail_offset, 0.0f, 0.0f),
                new Vector3f(5.31f, 3.10f, 0.10f), plane_mass*1/10));

        list.add(new Cuboid(new Vector3f(0f),
                new Vector3f(8,2,2), plane_mass/2));

        inertia = InertiaCalculator.getInertiaTensor(list, true);

        System.err.println(inertia);

        rb1.setEnableAirResistance(true);
        rb1.setSurfaceArea(15);
        for(int i = 0; i < 500; i ++){
            rb1.update(0.1f*i);
            System.err.println(rb1);
        }
    }

    static  Vector3f toDeg(Quaternionf quat){
        Vector3f vec = quat.getEulerAnglesXYZ(new Vector3f());

        vec.x = Math.toDegrees(vec.x);
        vec.y = Math.toDegrees(vec.y);
        vec.z = Math.toDegrees(vec.z);

        return vec;

    }


}

