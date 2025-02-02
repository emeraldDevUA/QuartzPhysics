package org.tourmaline.Collision;

import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class CollisionUtils {



    public static List<BoundingBox> approximateModel(List<Vector3f> vertices, int maxBoxes) {
        // Step 1: Cluster the model using K-means with a fixed number of clusters (maxBoxes)
        List<List<Vector3f>> clusters = clusterVerticesKMeans(vertices, maxBoxes);

        // Step 2: For each cluster, compute the OBB
        List<BoundingBox> obbs = new ArrayList<>();
        for (List<Vector3f> cluster : clusters) {
            BoundingBox obb = computeOBB(cluster);
            obbs.add(obb);
        }

        return obbs;
    }
    private static List<List<Vector3f>> clusterVerticesKMeans(List<Vector3f> vertices, int k) {
        // Initialize centroids randomly
        List<Vector3f> centroids = new ArrayList<>();
        Random random = new Random();
        for (int i = 0; i < k; i++) {
            centroids.add(new Vector3f(vertices.get(random.nextInt(vertices.size()))));
        }

        List<List<Vector3f>> clusters = new ArrayList<>();
        for (int i = 0; i < k; i++) clusters.add(new ArrayList<>());

        boolean changed;
        do {
            // Clear previous cluster assignments
            for (List<Vector3f> cluster : clusters) cluster.clear();

            // Assign each vertex to the nearest centroid
            for (Vector3f vertex : vertices) {
                int closest = 0;
                float minDistance = vertex.distanceSquared(centroids.get(0));
                for (int i = 1; i < centroids.size(); i++) {
                    float distance = vertex.distanceSquared(centroids.get(i));
                    if (distance < minDistance) {
                        closest = i;
                        minDistance = distance;
                    }
                }
                clusters.get(closest).add(vertex);
            }

            // Update centroids
            changed = false;
            for (int i = 0; i < k; i++) {
                if (clusters.get(i).isEmpty()) continue; // Skip empty clusters
                Vector3f newCentroid = new Vector3f();
                for (Vector3f vertex : clusters.get(i)) newCentroid.add(vertex);
                newCentroid.div(clusters.get(i).size());
                if (!newCentroid.equals(centroids.get(i))) {
                    centroids.set(i, newCentroid);
                    changed = true;
                }
            }
        } while (changed); // Repeat until centroids stabilize

        return clusters;
    }
    private static BoundingBox computeOBB(List<Vector3f> vertices) {
        // Step 1: Compute the center of the points
        Vector3f center = new Vector3f();
        for (Vector3f vertex : vertices) {
            center.add(vertex);
        }
        center.div(vertices.size());

        // Step 2: Compute the covariance matrix for the points
        Matrix3f covariance = new Matrix3f();
        for (Vector3f vertex : vertices) {
            Vector3f diff = new Vector3f(vertex).sub(center);
            covariance.m00 += diff.x * diff.x;
            covariance.m01 += diff.x * diff.y;
            covariance.m02 += diff.x * diff.z;
            covariance.m10 += diff.y * diff.x;
            covariance.m11 += diff.y * diff.y;
            covariance.m12 += diff.y * diff.z;
            covariance.m20 += diff.z * diff.x;
            covariance.m21 += diff.z * diff.y;
            covariance.m22 += diff.z * diff.z;
        }

        // Step 3: Find the eigenvectors of the covariance matrix (this gives the orientation of the OBB)
        EigenDecomposition eigenDecomposition = new EigenDecomposition(
                convert2RealMatrix(covariance));
        Matrix3f rotationMatrix =
                new Matrix3f(convertDouble2Float(
                        eigenDecomposition.getRealEigenvalues()));

        // Step 4: Project the points onto the eigenvectors to get the extents
        List<Vector3f> projectedVertices = new ArrayList<>();
        for (Vector3f vertex : vertices) {
            Vector3f projection = new Vector3f(vertex).sub(center).mul(rotationMatrix);
            projectedVertices.add(projection);
        }

        // Step 5: Find the extents (half-extents) along each axis
        float minX = Float.MAX_VALUE, maxX = Float.MIN_VALUE;
        float minY = Float.MAX_VALUE, maxY = Float.MIN_VALUE;
        float minZ = Float.MAX_VALUE, maxZ = Float.MIN_VALUE;

        for (Vector3f proj : projectedVertices) {
            minX = Math.min(minX, proj.x);
            maxX = Math.max(maxX, proj.x);
            minY = Math.min(minY, proj.y);
            maxY = Math.max(maxY, proj.y);
            minZ = Math.min(minZ, proj.z);
            maxZ = Math.max(maxZ, proj.z);
        }

        // The half-extents are half the length of the projections' min/max range
        Vector3f halfExtents = new Vector3f((maxX - minX) / 2, (maxY - minY) / 2, (maxZ - minZ) / 2);

        // Return the OBB
        return new BoundingBox(center, halfExtents,
                rotationMatrix.getNormalizedRotation(new Quaternionf()));
    }

    private static FloatBuffer convertDouble2Float(double[] realEigenvalues) {
        float[] array = new float[realEigenvalues.length];
        int i = 0;
        for(double t: realEigenvalues){
            array[i] = (float) t; i++;
        }
        return FloatBuffer.wrap(array);
    }

    private static RealMatrix convert2RealMatrix(Matrix3f jomlMatrix) {
        double[][] data = {
                { jomlMatrix.m00(), jomlMatrix.m01(), jomlMatrix.m02() },
                { jomlMatrix.m10(), jomlMatrix.m11(), jomlMatrix.m12() },
                { jomlMatrix.m20(), jomlMatrix.m21(), jomlMatrix.m22() }
        };
        // Create and return an Apache Commons Math RealMatrix
        return new BlockRealMatrix(data);
    }
}
