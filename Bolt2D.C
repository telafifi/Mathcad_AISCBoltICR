#include "Bolt2D.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>

/* Add the bolt to the linkedList
    */
    Bolt2D* add_bolt(Bolt2D* list, double X, double Y, double Z, double Ab, double Fnv) {
		Bolt2D* newVector = (Bolt2D*)malloc(sizeof(Bolt2D));
		if (newVector == NULL){
			return NULL;
		}
		newVector->X = X;
		newVector->Y = Y;
		newVector->Z = Z;
		newVector->Ab = Ab;
		newVector->Fnv = Fnv;
		newVector->next = NULL;

		//check if this is the first value in the linkedlist
		if (list == NULL){
			return (newVector);
		}

		//otherwise add it to the end of the list
		Bolt2D* head = list;
		while (list->next != NULL){
			list = list->next;
		}

		list->next = newVector;
		return (head);
	}

	/* Free all malloc'd values in the bolt list
    */
    void free_bolt_list(Bolt2D* list) {
		while (list != NULL){
			Bolt2D* next = list->next;
			free(list);
			list = next;
		}
	}

/* Get the angle (in radians) from 0 to 2*PI from an X and Y coordinate
    */
   double GetAngleFromComponents(double X, double Y){
       if (X == 0)
			{
				if (Y == 0)
				{
					return 0;
				}
				if (Y > 0)
				{
					return 0.5 * M_PI;
				}
				return -0.5 * M_PI;
			}
			double baseAngle = atan2(_abs64(Y), _abs64(X));
			if (Y < 0)
			{
				if (X > 0)
				{
					return 2 * M_PI - baseAngle;
				}
				return M_PI + baseAngle;
			}
			if (X < 0)
			{
				return M_PI - baseAngle;
			}
			return baseAngle;
   }

   /* Get the bolt group centroid 
    */
    void BoltGroupCentroid(Bolt2D* boltList, double* X, double* Y) {
        double Xcen = 0;
        double Ycen = 0;
        double SumAb = 0;
		Bolt2D* list = boltList;
        while (list != NULL){
            Xcen += list->Ab * list->X;
            Ycen += list->Ab * list->Y;
            SumAb += list->Ab;
			list = list->next;
        }
        *X = Xcen / SumAb;
        *Y = Ycen / SumAb;
    }