package org.tourmaline.Processing;

import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.joml.Matrix3f;
import org.joml.Vector3f;
import org.tourmaline.Collision.BoundingBox;
import org.tourmaline.Collision.CollisionPrimitive;
import org.tourmaline.RigidBody.RigidBody;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import java.util.concurrent.TimeUnit;

@RequiredArgsConstructor
public class PhysicsProcessor extends Thread{
    private static final int workerCount = 8;
    private final List<RigidBody> rigidBodies;
    private final ExecutorService executor =
            Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());

    private final float dt;
    @Setter
    public boolean isRunning = true;



    @Override
    public void run(){
        long t1, t2;
        t1 = System.nanoTime();
        while(isRunning) {

            t2 = System.nanoTime();
            if (t2 - t1 >= 16_666_66) {

                executor.submit(this::processBodyCollision);

                t1 = t2;
            }
        }

        executor.shutdown();
        try {
            if (!executor.awaitTermination(5, TimeUnit.SECONDS)) {
                executor.shutdownNow(); // Force shutdown if not terminated
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

                    if(j == i){continue;}
                    RigidBody firstBody = rigidBodies.get(i);
                    RigidBody secondBody = rigidBodies.get(j);

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
                        collideBodies(firstBody, secondBody);



                    }
                }
            }
    }

    void collideBodies(RigidBody b1, RigidBody b2){
        // body 1 - this
        Vector3f w1 = b1.getAngularVelocity().get(new Vector3f());
        Vector3f velocity1 = b1.getVelocity().get(new Vector3f());

        Matrix3f I1 = new Matrix3f(b1.getInertia());
        float m1 = b1.getMass();


        // body 2 - other

        Vector3f w2 = b2.getAngularVelocity().get(new Vector3f());
        Vector3f velocity2 = b2.getVelocity().get(new Vector3f());
        Matrix3f I2 = new Matrix3f(b2.getInertia());
        float m2 = b2.getMass();

        Vector3f lineOfCollision = new Vector3f(b1.getPosition()).sub(b2.getPosition());
        Vector3f normalizedL = lineOfCollision.normalize(new Vector3f());

        // Calculate initial parallel components
        Vector3f parallelComponentsV1 = new Vector3f(normalizedL).mul(velocity1.dot(normalizedL));
        Vector3f parallelComponentsV2 = new Vector3f(normalizedL).mul(velocity2.dot(normalizedL));

        // Calculate new parallel components after collision
        Vector3f finalParallelV1 = new Vector3f(parallelComponentsV1)
                .mul(m1 - m2)
                .add(parallelComponentsV2.mul(2 * m2, new Vector3f()))
                .div(m1 + m2);

        Vector3f finalParallelV2 = new Vector3f(parallelComponentsV2)
                .mul(m2 - m1)
                .add(parallelComponentsV1
                        .mul(2 * m1, new Vector3f()))
                .div(m1 + m2);


        // Calculate perpendicular components (they remain the same)
        Vector3f perpendicularComponentsV1 = velocity1.sub(parallelComponentsV1, new Vector3f());
        Vector3f perpendicularComponentsV2 = velocity2.sub(parallelComponentsV2, new Vector3f());

        // Final velocities after collision
        Vector3f finalVelocity1 = new Vector3f(perpendicularComponentsV1).add(finalParallelV1);
        Vector3f finalVelocity2 = new Vector3f(perpendicularComponentsV2).add(finalParallelV2);

        b1.setVelocity(finalVelocity1);
        b2.setVelocity(finalVelocity2);

        Vector3f M1 = w1.mul(I1, new Vector3f());
        Vector3f M2 = w2.mul(I2, new Vector3f());

        Vector3f MSum = M1.add(M2, new Vector3f());

        Matrix3f I11 = I1.add(I1, new Matrix3f());
        Matrix3f I22 = I2.add(I2, new Matrix3f());

        Vector3f _w1 = MSum.mul(I22.invert(), new Vector3f());
        Vector3f _w2 = MSum.mul(I11.invert(), new Vector3f());

        Matrix3f mat = I1.sub(I2, new Matrix3f());


        float[] conditions = {mat.m00, mat.m11, mat.m22};

        // Apply the function for each condition and corresponding component
        adjustComponent(conditions[0], _w1, _w2, 'x');
        adjustComponent(conditions[1], _w1, _w2, 'y');
        adjustComponent(conditions[2], _w1, _w2, 'z');

        b1.setAngularVelocity(_w1);
        b2.setAngularVelocity(_w2);

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
    }
}


