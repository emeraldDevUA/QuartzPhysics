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
    private boolean enableLift = false;
    private boolean enableNormalReaction = false;
    private boolean enableFriction = false;

    private float frictionQuotient;
    private float surfaceArea;

    private Vector3f surfaceNormal;
    private DampingFunctions dampingFunctions;
    private Airfoil airFoil;

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
                if(enableLift){
                    Vector3f normal = new Vector3f(0,1,0);
                    normal.mul(
                            0.5f*po*squaredSpeed*surfaceArea*tuple.a
                    );
                    normal.div(mass);
                    acceleration.add(normal);
                }


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

        Vector3f transform = inertia.transform(new Vector3f(angularVelocity));
        Vector3f torqueDifference = new Vector3f(netTorque).sub(
                new Vector3f(angularVelocity).cross(transform));

        Vector3f finalBeta = inverseInertia.transform(torqueDifference).mul(dt);
        angularVelocity.add(finalBeta);

        if (angularVelocity.length() > MAX_ANGULAR_VELOCITY) {
            angularVelocity.normalize().mul(MAX_ANGULAR_VELOCITY);
        }

//        Quaternionf deltaRotation
//                = new Quaternionf(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0)
//                .mul(dt / 2);
//        orientation.add(deltaRotation);

        if(dampingFunctions != null) {
            angularVelocity.mul(dampingFunctions.getAngularVelocityDamping(dt));
        }

        orientation.rotateLocalX(angularVelocity.x*dt/2);
        orientation.rotateLocalY(angularVelocity.y*dt/2);
        orientation.rotateLocalZ(angularVelocity.z*dt/2);

        orientation.normalize();

        // Reset forces and torques for next iteration
        netForce.set(0);
        netTorque.set(0);
    }
    private Vector3f computeAngularAcceleration(Vector3f omega) {
        Vector3f Iomega = inertia.transform(new Vector3f(omega));
        Vector3f torqueTerm = new Vector3f(omega).cross(Iomega);
        Vector3f accel = new Vector3f(netTorque).sub(torqueTerm);
        return inverseInertia.transform(accel);
    }

     public void rk_update(float dt){
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
                if(enableLift){
                    Vector3f normal = new Vector3f(0,1,0);
                    normal.mul(
                            0.5f*po*squaredSpeed*surfaceArea*tuple.a
                    );
                    normal.div(mass);
                    acceleration.add(normal);
                }


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

        Vector3f acceleration = new Vector3f(netForce); // divided by mass if needed

// RK4 for velocity
        Vector3f k1v = new Vector3f(acceleration);
        Vector3f k2v = new Vector3f(acceleration);
        Vector3f k3v = new Vector3f(acceleration);
        Vector3f k4v = new Vector3f(acceleration);

        Vector3f deltaV = new Vector3f(k1v).add(new Vector3f(k2v).mul(2)).add(new Vector3f(k3v).mul(2)).add(k4v).mul(dt / 6f);
        velocity.add(deltaV);

// Cap velocity
        if (velocity.length() > MAX_VELOCITY) {
            velocity.normalize().mul(MAX_VELOCITY);
        }

// RK4 for position
        Vector3f k1x = new Vector3f(velocity);
        Vector3f k2x = new Vector3f(velocity).add(new Vector3f(deltaV).mul(0.5f));
        Vector3f k3x = new Vector3f(velocity).add(new Vector3f(deltaV).mul(0.5f));
        Vector3f k4x = new Vector3f(velocity).add(new Vector3f(deltaV));

        Vector3f deltaX = new Vector3f(k1x).add(new Vector3f(k2x).mul(2)).add(new Vector3f(k3x).mul(2)).add(k4x).mul(dt / 6f);
        position.add(deltaX);

        Vector3f transform = inertia.transform(new Vector3f(angularVelocity));
        Vector3f torqueDifference = new Vector3f(netTorque).sub(
                new Vector3f(angularVelocity).cross(transform));

        Vector3f finalBeta = inverseInertia.transform(torqueDifference).mul(dt);
        angularVelocity.add(finalBeta);

        Vector3f k1w = computeAngularAcceleration(angularVelocity);
        Vector3f k2w = computeAngularAcceleration(new Vector3f(angularVelocity).add(new Vector3f(k1w).mul(dt / 2f)));
        Vector3f k3w = computeAngularAcceleration(new Vector3f(angularVelocity).add(new Vector3f(k2w).mul(dt / 2f)));
        Vector3f k4w = computeAngularAcceleration(new Vector3f(angularVelocity).add(new Vector3f(k3w).mul(dt)));

        Vector3f deltaOmega = new Vector3f(k1w).add(new Vector3f(k2w).mul(2)).add(new Vector3f(k3w).mul(2)).add(k4w).mul(dt / 6f);
        angularVelocity.add(deltaOmega);

// Cap angular velocity
        if (angularVelocity.length() > MAX_ANGULAR_VELOCITY) {
            angularVelocity.normalize().mul(MAX_ANGULAR_VELOCITY);
        }

// Damping
        if (dampingFunctions != null) {
            angularVelocity.mul(dampingFunctions.getAngularVelocityDamping(dt));
        }

        Quaternionf omegaQuat = new Quaternionf(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0f);

        Quaternionf q0 = new Quaternionf(orientation);
        Quaternionf k1q = new Quaternionf(q0).mul(omegaQuat).mul(0.5f);

        Quaternionf q1 = new Quaternionf(q0).add(new Quaternionf(k1q).mul(dt / 2f));
        Quaternionf k2q = new Quaternionf(q1).mul(omegaQuat).mul(0.5f);

        Quaternionf q2 = new Quaternionf(q0).add(new Quaternionf(k2q).mul(dt / 2f));
        Quaternionf k3q = new Quaternionf(q2).mul(omegaQuat).mul(0.5f);

        Quaternionf q3 = new Quaternionf(q0).add(new Quaternionf(k3q).mul(dt));
        Quaternionf k4q = new Quaternionf(q3).mul(omegaQuat).mul(0.5f);

        Quaternionf deltaQ = new Quaternionf(k1q)
                .add(new Quaternionf(k2q).mul(2f))
                .add(new Quaternionf(k3q).mul(2f))
                .add(k4q)
                .mul(dt / 6f);

        orientation.add(deltaQ);
        orientation.normalize();
    }    /*
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

    public void applyTorque(Vector3f force) {
        netTorque.add(force) ;
    }
    public void applyTorque(Vector3f force, Vector3f point) {
        netTorque.add(new Vector3f(point).cross(force)) ;
    }
    /**
     * Assumes you apply forces in body space, taking no account for rotations explicitly
     */
    public void applyForceAtPoint(Vector3f force, Vector3f point){


        netForce.add(transformDirection(force));
        netTorque.add(new Vector3f(point).cross(force));
    }


    public void setInertiaTensor(Matrix3f inertiaTensor){
        this.inertia = inertiaTensor;
        this.inverseInertia = new Matrix3f(inertiaTensor).invert();
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
