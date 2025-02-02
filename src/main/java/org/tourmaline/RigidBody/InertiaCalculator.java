package org.tourmaline.RigidBody;

import lombok.Getter;
import org.joml.Matrix3f;
import org.joml.Vector3f;
import org.tourmaline.RigidBody.InertiaPrimitives.InertiaPrimitive;

import java.util.List;

import static java.lang.Math.pow;


public class InertiaCalculator {
    @Getter
    private static Vector3f centerOfGravity;


    public static Matrix3f getInertiaTensor(List<InertiaPrimitive> inertiaElements, boolean precomputed_offset){
        float Ixx = 0, Iyy = 0, Izz = 0;
        float Ixy = 0, Ixz = 0, Iyz = 0;



        Vector3f torque = new Vector3f(0f);

        float[] total_mass = {0f};
        inertiaElements.forEach(inertiaPrimitive -> {
            total_mass[0] += inertiaPrimitive.getMass();
            torque.add(
                    new Vector3f(inertiaPrimitive.getPosition())
                            .mul(inertiaPrimitive.getMass()));
        });

        centerOfGravity = torque.div(total_mass[0]);

        for(InertiaPrimitive primitive : inertiaElements ) {

            if (!precomputed_offset) {
                primitive.setOffset(
                        new Vector3f(primitive.getPosition()).sub(centerOfGravity));
            } else {
                primitive.setOffset(primitive.getPosition());
            }

            Vector3f offset = primitive.getOffset();

            Ixx += (float) (primitive.getInertia().x + primitive.getMass()
                                * (pow(offset.y, 2) + pow(offset.z, 2)));

            Iyy += (float) (primitive.getInertia().y + primitive.getMass()
                    * (pow(offset.z, 2) + pow(offset.x, 2)));

            Izz += (float) (primitive.getInertia().z + primitive.getMass()
                    * (pow(offset.x, 2) + pow(offset.y, 2)));

            Ixy += primitive.getMass() * (offset.x * offset.y);
            Ixz += primitive.getMass() * (offset.x * offset.z);
            Iyz += primitive.getMass() * (offset.y * offset.z);

        }


        return new Matrix3f(Ixx, -Ixy, -Ixz,
                           -Ixy,  Iyy, -Iyz,
                           -Ixz, -Iyz,  Izz);
    }

}
