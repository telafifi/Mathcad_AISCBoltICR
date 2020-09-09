///Define a 3D vector structure
    typedef struct Bolt2D_struct {
        double X;
        double Y;
        double Z;
        double Ab;
        double Fnv;
        struct Bolt2D_struct* next;
    } Bolt2D ;

    /* Add the bolt to the linkedList
    */
    Bolt2D* add_bolt(Bolt2D* list, double X, double Y, double Z, double Ab, double Fnv) ;

    /* Free all malloc'd values in the bolt list
    */
    void free_bolt_list(Bolt2D* list) ;

    /* Get the bolt group centroid 
    */
    void BoltGroupCentroid(Bolt2D* boltList, double* X, double* Y) ;

    /* Get the angle (in radians) from 0 to 2*PI from an X and Y coordinate
    */
    double GetAngleFromComponents(double X, double Y);