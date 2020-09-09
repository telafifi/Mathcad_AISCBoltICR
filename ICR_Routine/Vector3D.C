#include "Vector3D.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>

    /* Create a new vector object
    */
    Vector3D* CreateVector(double X, double Y, double Z){
        Vector3D* newVector = (Vector3D*)malloc(sizeof(Vector3D));
        if (newVector == NULL){
            return NULL;
        }
        newVector->X = X;
        newVector->Y = Y;
        newVector->Z = Z;
        return newVector;
    }

    /* Return the magnitude of the vector from the point of the vector
    */
    double GetVectorMagnitude (Vector3D* vector){
        return sqrt(pow(vector->X, 2) + pow(vector->Y, 2) + pow(vector->Z, 2));
    }

    /* Multiply the vector with a constant and return a new malloc'd vector
     */ 
    Vector3D* MultiplyVector(double magnification, Vector3D* vector){
        return CreateVector(magnification * vector->X, magnification * vector->Y, magnification * vector->Z);
    }

    /* Return the unit vector of a vector as a malloc'd vector
    */
    Vector3D* GetUnitVector(Vector3D* vector){
        double magnitude = GetVectorMagnitude(vector);
        magnitude = 1.0 / magnitude;
        return MultiplyVector(magnitude, vector);
    }

    /* Return the cross product of two vectors
    */
    Vector3D* Cross(Vector3D* vector1, Vector3D* vector2){
        Vector3D* newVector = (Vector3D*)malloc(sizeof(Vector3D));
        if (newVector == NULL){
            return NULL;
        }
        newVector->X = vector1->Y * vector2->Z - vector1->Z * vector2->Y;
        newVector->Y = vector1->Z * vector2->X - vector1->X * vector2->Z;
        newVector->Z = vector1->X * vector2->Y - vector1->Y * vector2->X;
        return newVector;
    }

    /* Get Vector in Eccentric direction. Angle fed in is in radians
    */
   Vector3D* EccentricDirection(double angle){
       double eccentricAngle = angle + M_PI * 0.5;
       return CreateVector(cos(eccentricAngle), sin(eccentricAngle), 0);
   }

   /* Get the unit vector of the cross vector of 2 vectors and magnify it by a factor
    */
   Vector3D* MultiplyCrossUnitCombo(double magnification, Vector3D* vector1, Vector3D* vector2) {
       Vector3D* crossProduct = Cross(vector1, vector2);
       Vector3D* unitVector = GetUnitVector(crossProduct);
       Vector3D* result = MultiplyVector(magnification, unitVector);
       free(crossProduct);
       free(unitVector);
       return result;
   }

   