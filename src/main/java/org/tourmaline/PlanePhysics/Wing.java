package org.tourmaline.PlanePhysics;

import lombok.Getter;
import lombok.Setter;
import org.joml.Vector3f;
import org.tourmaline.PlanePhysics.Airfoil.Airfoil;
import org.tourmaline.RigidBody.RigidBody;


import static java.lang.StrictMath.*;
import static java.lang.StrictMath.PI;
import static org.tourmaline.PlanePhysics.Atmosphere.getAirDensity;

public class Wing implements ActiveElement{
    @Setter
    private float inputControl;
    private float flap_ratio;
    private float area;
    private float aspect_ratio, efficiency_factor;
    @Getter
    private Vector3f position;
    private Vector3f normal;
    private Airfoil airfoil;
    public Wing(Vector3f position, float span, float chord, Airfoil airfoil, Vector3f normal,
                float flap_ratio){
        this.normal = normal;
        this.airfoil = airfoil;
        this.position = position;
        this.flap_ratio = flap_ratio;
        this.efficiency_factor = 1;
        this.inputControl = 0.0f;

        this.area = span*chord;

        this.aspect_ratio = (float) (pow(span,2)/area);

    }

    @Override
    public Vector3f getForce(RigidBody rb) {
        Vector3f velocity = rb.getPointVelocity(new Vector3f(position));
        if (velocity.length() <= pow(10, -12)) {
            System.err.println("Velocity is equal to zero!");
            return new Vector3f(0);
        }
        Vector3f drag_direction = (new Vector3f(velocity).negate()).normalize();

        Vector3f lift_direction = new Vector3f();  // to store the final result
        drag_direction.cross(normal, lift_direction)  // first cross product
                .cross(drag_direction, lift_direction)  // second cross product
                .normalize();  // normalize the final result

        float angleOfAttack = (float) Math.toDegrees(Math.asin(drag_direction.dot(normal)));

        Tuple<Float, Float> ClCd = airfoil.sample(abs(angleOfAttack));

        if (flap_ratio > 0.0f) {
            // lift coefficient changes based on flap deflection ie control input
            float delta_lift_coeff = (float) (sqrt(flap_ratio) * airfoil.cl_max * inputControl);
            ClCd.a += delta_lift_coeff;
        }
        float induced_drag_coeff = (float) (pow(ClCd.a, 2) / (PI * aspect_ratio * efficiency_factor));
        ClCd.b += induced_drag_coeff;

        float air_density = getAirDensity(rb.getPosition().y);

        float dynamic_pressure = (float) (0.5f * pow(velocity.length(), 2) * air_density * area);

        Vector3f liftForce;
        Vector3f dragForce;

        liftForce = lift_direction.mul(ClCd.a, new Vector3f());
        dragForce = drag_direction.mul(ClCd.b, new Vector3f());

        liftForce = liftForce.mul(dynamic_pressure);
        dragForce = dragForce.mul(dynamic_pressure);

        rb.applyForceAtPoint(liftForce.add(dragForce, new Vector3f()), position);

        return liftForce.add(dragForce, new Vector3f());
    }

}
