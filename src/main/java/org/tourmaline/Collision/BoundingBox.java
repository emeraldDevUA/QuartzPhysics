package org.tourmaline.Collision;

import lombok.Getter;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.Serializable;

@Getter

public class BoundingBox extends CollisionPrimitive implements Serializable {
    private Vector3f halfDims;

    public BoundingBox(final Vector3f position, final Vector3f halfDims,
                       final Quaternionf quaternion ){
        setPosition(position);
        this.halfDims = halfDims;
        setQuaternion(quaternion);



    }


    private float getProjectionRadius(Vector3f axis, Matrix3f rotationMatrix) {
        // Радиус проекции это сумма проекций половинных размеров по всем осям
        Vector3f xAxis = new Vector3f();
        Vector3f yAxis = new Vector3f();
        Vector3f zAxis = new Vector3f();
        rotationMatrix.getColumn(0, xAxis);  // Получаем ось X коробки
        rotationMatrix.getColumn(1, yAxis);  // Получаем ось Y коробки
        rotationMatrix.getColumn(2, zAxis);  // Получаем ось Z коробки

        // Скалярные произведения осей с вектором для получения проекций
        return  halfDims.x * Math.abs(axis.dot(xAxis)) +
                halfDims.y * Math.abs(axis.dot(yAxis)) +
                halfDims.z * Math.abs(axis.dot(zAxis));
    }

    // Метод для проверки пересечения на одной оси
    private boolean testAxis(Vector3f axis, BoundingBox other, Vector3f t, Matrix3f rotationA, Matrix3f rotationB) {
        if (axis.lengthSquared() < 1e-6f) return true;  // Skip near-zero axis
        axis.normalize();

        float ra = getProjectionRadius(axis, rotationA);
        float rb = other.getProjectionRadius(axis, rotationB);
        float distance = Math.abs(t.dot(axis));

        return distance <= ra + rb + 1e-5f;  // Add small epsilon to account for floating-point errors
    }
    public boolean checkCollision(CollisionPrimitive other) {
        Matrix3f rotationA = getRotationMatrix();
        Matrix3f rotationB = other.getRotationMatrix();

        // Вектор между центрами коробок
        Vector3f t = new Vector3f(other.getPosition()).sub(this.getPosition());

        // Преобразуем в локальные координаты первой коробки
        t = new Vector3f(
                t.dot(rotationA.getColumn(0, new Vector3f())),
                t.dot(rotationA.getColumn(1, new Vector3f())),
                t.dot(rotationA.getColumn(2, new Vector3f()))
        );

        // Оси для проверки (сначала оси обеих коробок)
        Vector3f[] axes = new Vector3f[15];

        // Оси коробки A
        axes[0] = rotationA.getColumn(0, new Vector3f());
        axes[1] = rotationA.getColumn(1, new Vector3f());
        axes[2] = rotationA.getColumn(2, new Vector3f());

        // Оси коробки B
        axes[3] = rotationB.getColumn(0, new Vector3f());
        axes[4] = rotationB.getColumn(1, new Vector3f());
        axes[5] = rotationB.getColumn(2, new Vector3f());

        // Перекрёстные оси (A_i x B_j)
        int index = 6;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                axes[index] = new Vector3f(axes[i]).cross(axes[3 + j]);

                // Ensure cross-product axis is not too small
                if (axes[index].lengthSquared() < 1e-6f) {
                    axes[index].set(0, 0, 0); // Mark as invalid
                } else {
                    axes[index].normalize();
                }

                index++;
            }
        }

        // Проверяем все 15 осей
        for (Vector3f axis : axes) {
            if (axis.lengthSquared() < 1e-6f) continue;  // Пропускаем почти нулевые оси

            if (!testAxis(axis, (BoundingBox) other, t, rotationA, rotationB)) {
                return false;  // Нашли разделяющую ось, столкновения нет
            }
        }

        // Если разделяющая ось не найдена, значит, коробки пересекаются
        return true;
    }



}
