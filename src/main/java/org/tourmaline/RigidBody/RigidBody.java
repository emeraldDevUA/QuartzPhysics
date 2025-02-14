package org.tourmaline.RigidBody;

import lombok.Getter;
import lombok.Setter;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.tourmaline.Collision.CollisionPrimitive;
import org.tourmaline.PlanePhysics.Airfoil.Airfoil;
import org.tourmaline.PlanePhysics.Tuple;

import static org.tourmaline.PlanePhysics.Airfoil.Airfoil.arrayToList;
import static org.tourmaline.PlanePhysics.Airfoil.Constants.NACA_2412;

@Getter
@Setter
public class RigidBody {
    private static final float MAX_ANGULAR_VELOCITY = 20;
    private static final float MAX_VELOCITY = 1000;
    private static final float EPSILON = 10E-6f;
    // unused for now
    private CollisionPrimitive collisionPrimitive;
    private float mass;

    private Vector3f position;
    @Getter
    private Vector3f velocity, acceleration, angularVelocity;
    @Getter
    private Vector3f netForce, netTorque;

    // Inertia tensor for the rigid body.
    private Matrix3f inertia, inverseInertia;

    // Integral of angular velocity
    @Getter
    private Quaternionf orientation;

    private boolean enableGravity = true;
    private boolean enableAirResistance = false;
    private boolean enableNormalReaction = false;
    private boolean enableFriction = false;

    private float frictionQuotient;
    private float surfaceArea;

    private Vector3f surfaceNormal;

    private Airfoil airFoil = null;

    public RigidBody(Matrix3f inertia, Vector3f position, float mass){
        this.mass = mass;
        this.inertia = new Matrix3f(inertia);
        this.inverseInertia = new Matrix3f(inertia).invert();
        this.position = position;

        velocity = new Vector3f(0);
        acceleration = new Vector3f(0);
        angularVelocity = new Vector3f(0);

        netForce = new Vector3f(0);
        netTorque = new Vector3f(0);

        orientation = new Quaternionf(0,0,0,1);

        frictionQuotient = 0.4f;
        surfaceNormal = new Vector3f(0,1,0);


        airFoil = new Airfoil(arrayToList(NACA_2412));
    }


    public void update(float dt){
        acceleration = netForce.div(mass, new Vector3f());

        if(enableGravity){
            acceleration.y -= 9.8f;
        }

        if(enableAirResistance){
            // compute air resistance
            float po = 1.225f;
            float squaredSpeed = velocity.lengthSquared();
            if(!(squaredSpeed <= EPSILON)) {

                Vector3f reversedVelocity = new Vector3f(velocity)
                        .negate().normalize();

                Vector3f bodyNormal = new Vector3f(0,1,0);

                float angle = (float) Math.toDegrees(
                        Math.asin(reversedVelocity.dot(bodyNormal))
                );
                Tuple<Float, Float> tuple = airFoil.sample(angle);
                // Compute air resistance force: 1/2 * p * v^2 * S * Cd * -v̂

                reversedVelocity.mul(
                        0.5f*po*squaredSpeed*surfaceArea*tuple.b
                );
                reversedVelocity.div(mass);
                acceleration.add(reversedVelocity);
                // use some airfoil to compute drag.

            }
        }

        if(enableNormalReaction){
            Vector3f normalReactionForce = computeNormalReaction();
            normalReactionForce.div(mass);
            if (enableFriction) {
                float absVelocity = velocity.length();
                if(!(absVelocity <= EPSILON)){
                    Vector3f reversedVelocity = new Vector3f(velocity)
                            .negate().normalize();
                    Vector3f frictionForce =
                            reversedVelocity.mul(frictionQuotient*normalReactionForce.length());
                    acceleration.add(frictionForce);
                }

            }
            acceleration.add(normalReactionForce);
        }


        position.add(new Vector3f(velocity).mul(dt))
                .add(new Vector3f(acceleration).mul(0.5f * dt * dt));

        velocity.add(new Vector3f(acceleration).mul(dt)); // Assuming acceleration was updated
        if (velocity.length() > MAX_VELOCITY) {
            velocity.normalize().mul(MAX_VELOCITY);
        }

        Vector3f torqueDifference = new Vector3f(netTorque)
                .sub(new Vector3f(angularVelocity)
                        .cross(inertia.transform(angularVelocity, new Vector3f())));

        Vector3f finalBeta = inverseInertia.transform(torqueDifference).mul(dt);
        angularVelocity.add(finalBeta);

        if (angularVelocity.length() > MAX_ANGULAR_VELOCITY) {
            angularVelocity.normalize().mul(MAX_ANGULAR_VELOCITY);
        }

//        Quaternionf deltaRotation
//                = new Quaternionf(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0)
//                .mul(dt / 2);
//        orientation.add(deltaRotation);


        orientation.rotateLocalX(angularVelocity.x*dt/2);
        orientation.rotateLocalY(angularVelocity.y*dt/2);
        orientation.rotateLocalZ(angularVelocity.z*dt/2);

        orientation.normalize();

        // Reset forces and torques for next iteration
        netForce.set(0);
        netTorque.set(0);
    }

    /*
    public void update(float dt) {
        // RK4 for Linear Dynamics
        Vector3f k1v = new Vector3f(), k2v = new Vector3f(), k3v = new Vector3f(), k4v = new Vector3f();
        Vector3f k1x = new Vector3f(), k2x = new Vector3f(), k3x = new Vector3f(), k4x = new Vector3f();

        Vector3f acceleration = new Vector3f();
        if (mass > 0) {
            acceleration.set(netForce).div(mass);
            if (enableGravity) {
                acceleration.y -= 9.8f;
            }
        }

        // k1
        k1v.set(acceleration).mul(dt);
        k1x.set(velocity).mul(dt);

        // k2
        Vector3f tempVelocity = new Vector3f(velocity).add(k1v.mul(0.5f));
        Vector3f tempPosition = new Vector3f(position).add(k1x.mul(0.5f));
        Vector3f tempAcceleration = new Vector3f(netForce).div(mass);
        if (enableGravity) {
            tempAcceleration.y -= 9.8f;
        }
        k2v.set(tempAcceleration).mul(dt);
        k2x.set(tempVelocity).mul(dt);

        // k3
        tempVelocity.set(velocity).add(k2v.mul(0.5f));
        tempPosition.set(position).add(k2x.mul(0.5f));
        tempAcceleration.set(netForce).div(mass);
        if (enableGravity) {
            tempAcceleration.y -= 9.8f;
        }
        k3v.set(tempAcceleration).mul(dt);
        k3x.set(tempVelocity).mul(dt);

        // k4
        tempVelocity.set(velocity).add(k3v);
        tempPosition.set(position).add(k3x);
        tempAcceleration.set(netForce).div(mass);
        if (enableGravity) {
            tempAcceleration.y -= 9.8f;
        }
        k4v.set(tempAcceleration).mul(dt);
        k4x.set(tempVelocity).mul(dt);

        // Update linear state
        velocity.add(new Vector3f(k1v).add(k2v.mul(2)).add(k3v.mul(2)).add(k4v).mul(1 / 6f));
        position.add(new Vector3f(k1x).add(k2x.mul(2)).add(k3x.mul(2)).add(k4x).mul(1 / 6f));

        // RK4 for Angular Dynamics
        Vector3f k1w = new Vector3f(), k2w = new Vector3f(), k3w = new Vector3f(), k4w = new Vector3f();
        Quaternionf deltaRotation = new Quaternionf();

        Vector3f torqueDifference = new Vector3f(netTorque)
                .sub(angularVelocity.cross(inertia.transform(angularVelocity, new Vector3f())));
        Vector3f angularAcceleration = inverseInertia.transform(torqueDifference);

        // k1
        k1w.set(angularAcceleration).mul(dt);

        // k2
        tempVelocity.set(angularVelocity).add(k1w.mul(0.5f));
        torqueDifference = new Vector3f(netTorque)
                .sub(tempVelocity.cross(inertia.transform(tempVelocity, new Vector3f())));
        angularAcceleration = inverseInertia.transform(torqueDifference);
        k2w.set(angularAcceleration).mul(dt);

        // k3
        tempVelocity.set(angularVelocity).add(k2w.mul(0.5f));
        torqueDifference = new Vector3f(netTorque)
                .sub(tempVelocity.cross(inertia.transform(tempVelocity, new Vector3f())));
        angularAcceleration = inverseInertia.transform(torqueDifference);
        k3w.set(angularAcceleration).mul(dt);

        // k4
        tempVelocity.set(angularVelocity).add(k3w);
        torqueDifference = new Vector3f(netTorque)
                .sub(tempVelocity.cross(inertia.transform(tempVelocity, new Vector3f())));
        angularAcceleration = inverseInertia.transform(torqueDifference);
        k4w.set(angularAcceleration).mul(dt);

        // Update angular state
        angularVelocity.add(new Vector3f(k1w).add(k2w.mul(2)).add(k3w.mul(2)).add(k4w).mul(1 / 6f));
        deltaRotation.set(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0).mul(dt / 2);
        orientation.add(deltaRotation);
        orientation.normalize();

        // Reset forces and torques for the next iteration
        netForce.set(0);
        netTorque.set(0);
    }
*/


    /**
     * Assumes you apply forces in body space, taking no account for rotations explicitly
     */
    public void applyForce(Vector3f force){
        netForce.add(transformDirection(force));
    }

    private Vector3f computeNormalReaction(){

        return new Vector3f(surfaceNormal).normalize()
                .mul(netForce.dot(surfaceNormal) /
                                surfaceNormal.lengthSquared());
    }

    public void applyTorque(Vector3f vector3f) {
        netTorque.add(vector3f);
    }
    /**
     * Assumes you apply forces in body space, taking no account for rotations explicitly
     */
    public void applyForceAtPoint(Vector3f force, Vector3f point){


        netForce.add(transformDirection(force));
        netTorque.add(new Vector3f(point).cross(force));
    }




    protected Vector3f transformDirection(Vector3f vector){
        return orientation.transform(new Vector3f(vector));
    }

    protected Vector3f transformDirectionInverse(Vector3f vector){
        return new Quaternionf(orientation).invert().transform(new Vector3f(vector));
    }


    public Vector3f getPointVelocity(Vector3f position) {
        Vector3f bodyVelocity = transformDirectionInverse(new Vector3f(velocity));
        Vector3f crossProduct = new Vector3f(angularVelocity).cross(position);
        bodyVelocity.add(crossProduct);

        return bodyVelocity;
    }

    @Override
    public String toString() {
        return STR."RigidBody{position=\{position}, velocity=\{velocity}, acceleration=\{acceleration}, angularVelocity=\{angularVelocity}, orientation=\{orientation}\{'}'}";
    }
}
