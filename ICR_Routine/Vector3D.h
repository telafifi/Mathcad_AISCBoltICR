///Define a 3D vector structure
    typedef struct Vector3D_struct {
        double X ;
        double Y;
        double Z;
    } Vector3D ;

    /* Create a new vector object
    */
    Vector3D* CreateVector(double X, double Y, double Z);

    /* Return the magnitude of the vector from the point of the vector
    */
    double GetVectorMagnitude (Vector3D* vector);

    /* Multiply the vector with a constant and return a new malloc'd vector
     */ 
    Vector3D* MultiplyVector(double magnification, Vector3D* vector);

    /* Return the unit vector of a vector as a malloc'd vector
    */
    Vector3D* GetUnitVector(Vector3D* vector);

    /* Return the cross product of two vectors
    */
   Vector3D* Cross(Vector3D* vector1, Vector3D* vector2);

    /* Get Vector in Eccentric direction
    */
   Vector3D* EccentricDirection(double angle);

    /* Get the unit vector of the cross vector of 2 vectors and magnify it by a factor
    */
   Vector3D* MultiplyCrossUnitCombo(double magnification, Vector3D* vector1, Vector3D* vector2) ;

    