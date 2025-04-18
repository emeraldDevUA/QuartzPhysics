package org.tourmaline.Processing;

import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.joml.Matrix3f;
import org.joml.Vector3f;
import org.tourmaline.Collision.BoundingBox;
import org.tourmaline.Collision.CollisionPrimitive;
import org.tourmaline.RigidBody.RigidBody;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import java.util.concurrent.TimeUnit;

@RequiredArgsConstructor
public class PhysicsProcessor extends Thread{

    private final List<RigidBody> rigidBodies;
    private final ExecutorService executor =
            Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());

    private final float dt;
    @Setter
    public boolean isRunning = true;



    @Override
    public void run() {
        final long targetFrameTime = 16_666_66L; // 60 FPS in nanoseconds
        long lastTime = System.nanoTime();

        while (isRunning) {
            long currentTime = System.nanoTime();
            long elapsedTime = currentTime - lastTime;

            if (elapsedTime >= targetFrameTime) {
                executor.submit(this::processBodyCollision);
                lastTime = currentTime;
            }

            // Prevent busy-waiting & high CPU usage
            try {
                long sleepTime = (targetFrameTime - elapsedTime) / 1_000_000; // Convert to milliseconds
                if (sleepTime > 0) {
                    Thread.sleep(sleepTime);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        // Shutdown executor safely
        executor.shutdown();
        try {
            if (!executor.awaitTermination(5, TimeUnit.SECONDS)) {
                executor.shutdownNow();
            }
        } catch (InterruptedException e) {
            executor.shutdownNow();
            Thread.currentThread().interrupt();
        }
    }



    private void processBodyCollision(){
            boolean[][] collisionMatrix = new boolean[rigidBodies.size()][rigidBodies.size()];
            for(int i = 0; i < rigidBodies.size(); i ++){
                for(int j = 0; j < rigidBodies.size(); j++){
//                    if(!collisionMatrix[i][j]){
//                        continue;
//                    }


                    RigidBody firstBody = rigidBodies.get(i);
                    RigidBody secondBody = rigidBodies.get(j);
                    if(firstBody.equals(secondBody)){continue;}

                    CollisionPrimitive firstCollision = firstBody.getCollisionPrimitive();
                    CollisionPrimitive secondCollision = secondBody.getCollisionPrimitive();

                    if(firstCollision instanceof BoundingBox bb1
                            && secondCollision instanceof BoundingBox bb2){

                        Vector3f tmp = bb1.getHalfDims();

                        // nasty approximation to cut out at least some excess collisions
                        float radius1 = (float) max(tmp.x, tmp.y, tmp.z);
                        tmp = bb2.getHalfDims();
                        float radius2 = (float) max(tmp.x, tmp.y, tmp.z);

                        if(bb1.getPosition().length() - bb2.getPosition().length() > radius1 + radius2){
                            collisionMatrix[i][j] = false;
                            collisionMatrix[j][i] = false;
                            continue;
                        }


                    }

                    collisionMatrix[i][j] =  firstCollision.checkCollision(secondCollision);
                    collisionMatrix[j][i] = collisionMatrix[i][j];
                    if(collisionMatrix[i][j]){
                        // execute lambda.
                        Runnable temp = rigidBodies.get(i).getCollisionPrimitive().getCollisionLambda();
                        if(temp != null){
                            executor.submit(temp);
                        }
                        Runnable temp2  = rigidBodies.get(j).getCollisionPrimitive().getCollisionLambda();
                        if(temp != null){
                            executor.submit(temp2);
                        }
                        if(firstBody.getCollisionPrimitive().isOn() && secondBody.getCollisionPrimitive().isOn()) {

                            collideBodies(firstBody, secondBody);

                            firstBody.update(dt);
                            secondBody.update(dt);
                        }

                    }
                }
            }
    }
/**/
    private void collideBodies(RigidBody b1, RigidBody b2) {

        // Extract mass and inertia properties
        float m1 = b1.getMass();
        float m2 = b2.getMass();
        Matrix3f I1 = new Matrix3f(b1.getInertia()).invert();
        Matrix3f I2 = new Matrix3f(b2.getInertia()).invert();

        // Positions and velocities
        Vector3f pos1 = b1.getPosition();
        Vector3f pos2 = b2.getPosition();
        Vector3f v1 = b1.getVelocity();
        Vector3f v2 = b2.getVelocity();
        Vector3f w1 = b1.getAngularVelocity();
        Vector3f w2 = b2.getAngularVelocity();

        // Collision normal (direction from b2 to b1)
        Vector3f collisionNormal = new Vector3f(pos1).sub(pos2).normalize();

        // Contact points (assuming contact occurs at midpoint for simplicity)
        Vector3f contactPoint = new Vector3f(pos1).add(pos2).mul(0.5f);
        Vector3f r1 = new Vector3f(contactPoint).sub(pos1);
        Vector3f r2 = new Vector3f(contactPoint).sub(pos2);

        // Relative velocity at contact point
        Vector3f relativeV = new Vector3f(v1).sub(v2)
                .add(new Vector3f(w1).cross(r1))
                .sub(new Vector3f(w2).cross(r2));

        // Compute impulse magnitude J
        float e = 0.9f; // Coefficient of restitution (elasticity, adjust as needed)
        float numer = -(1 + e) * relativeV.dot(collisionNormal);

        float denom = (1 / m1) + (1 / m2)
                + collisionNormal.dot(
                new Vector3f(I1.transform(new Vector3f(r1).cross(collisionNormal)))
                        .cross(r1)
        )
                + collisionNormal.dot(
                new Vector3f(I2.transform(new Vector3f(r2).cross(collisionNormal)))
                        .cross(r2)
        );

        float J = numer / denom;

        // Impulse vector
        Vector3f impulse = new Vector3f(collisionNormal).mul(J);

        // Apply impulses as forces at the contact point
        b1.applyForceAtPoint(new Vector3f(impulse), contactPoint);
        b2.applyForceAtPoint(new Vector3f(impulse).negate(), contactPoint);

    }


    void collideBodiesExact(RigidBody b1, RigidBody b2) {
        // Find collision points
        List<Vector3f> contactPoints = findCollisionPoints(
                (BoundingBox) b1.getCollisionPrimitive(),
                (BoundingBox) b2.getCollisionPrimitive());

        if (contactPoints.isEmpty()) return; // No collision detected

        float restitution = 0.8f; // Coefficient of restitution (elasticity)

        for (Vector3f contactPoint : contactPoints) {
            // Relative positions of contact points
            Vector3f r1 = new Vector3f(contactPoint).sub(b1.getPosition());
            Vector3f r2 = new Vector3f(contactPoint).sub(b2.getPosition());

            // Velocities at the contact points
            Vector3f v1 = new Vector3f(b1.getVelocity()).add(new Vector3f(b1.getAngularVelocity()).cross(r1));
            Vector3f v2 = new Vector3f(b2.getVelocity()).add(new Vector3f(b2.getAngularVelocity()).cross(r2));

            // Relative velocity at contact
            Vector3f relativeVelocity = new Vector3f(v1).sub(v2);

            // Collision normal (from b1 to b2)
            Vector3f normal = new Vector3f(b2.getPosition()).sub(b1.getPosition()).normalize();

            // Relative velocity along the normal
            float velAlongNormal = relativeVelocity.dot(normal);

            // Skip if objects are separating
            if (velAlongNormal > 0) continue;

            // Compute impulse scalar
            Matrix3f I1Inv = new Matrix3f(b1.getInertia()).invert();
            Matrix3f I2Inv = new Matrix3f(b2.getInertia()).invert();

            Vector3f r1CrossN = new Vector3f(r1).cross(normal);
            Vector3f r2CrossN = new Vector3f(r2).cross(normal);

            Vector3f I1r1CrossN = I1Inv.transform(r1CrossN);
            Vector3f I2r2CrossN = I2Inv.transform(r2CrossN);

            float impulseDenom = (1 / b1.getMass()) + (1 / b2.getMass()) +
                    normal.dot(new Vector3f(I1r1CrossN).cross(r1)) +
                    normal.dot(new Vector3f(I2r2CrossN).cross(r2));

            float impulseMagnitude = -(1 + restitution) * velAlongNormal / impulseDenom;
            Vector3f impulse = new Vector3f(normal).mul(impulseMagnitude);

            // Apply impulse to linear velocity
            b1.setVelocity(new Vector3f(b1.getVelocity()).add(new Vector3f(impulse).div(b1.getMass())));
            b2.setVelocity(new Vector3f(b2.getVelocity()).sub(new Vector3f(impulse).div(b2.getMass())));

            // Apply impulse to angular velocity
            Vector3f w1Change = I1Inv.transform(new Vector3f(r1).cross(impulse));
            Vector3f w2Change = I2Inv.transform(new Vector3f(r2).cross(impulse));

            b1.setAngularVelocity(new Vector3f(b1.getAngularVelocity()).add(w1Change));
            b2.setAngularVelocity(new Vector3f(b2.getAngularVelocity()).sub(w2Change));
        }
    }

    List<Vector3f> findCollisionPoints(BoundingBox boxA, BoundingBox boxB) {
        List<Vector3f> collisionPoints = new ArrayList<>();
        collisionPoints.add(new Vector3f(boxA.getPosition()).add(boxB.getPosition()).div(2));
        return collisionPoints;

    }

    private void adjustComponent(float condition, Vector3f w1, Vector3f w2, char component) {
        if (condition < 0) {
            applyAdjustment(w1, component);
        } else if (condition > 0) {
            applyAdjustment(w2, component);
        } else {
            applyAdjustment(w1, component);
            applyAdjustment(w2, component);
        }
    }
    private void applyAdjustment(Vector3f vector, char component) {
        switch (component) {
            case 'x':
                vector.x *= (float) -1;
                break;
            case 'y':
                vector.y *= (float) -1;
                break;
            case 'z':
                vector.z *= (float) -1;
                break;
        }
    }
    private double max(double a, double b, double c){
        if(a >= b && a >= c){
            return a;
        }
        if(b >= a && b >= c){
            return b;
        }
        return c;
    }


    public void addRigidBody(RigidBody rigidBody) {
        rigidBodies.add(rigidBody);
        System.err.println(rigidBodies.size());
    }

}


