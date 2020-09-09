#include "mcadincl.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include "Bolt2D.h"
#include "Vector3D.h"

#define  INTERRUPTED            1
#define  INSUFFICIENT_MEMORY    2
#define  MUST_BE_REAL           3
#define  NO_BOLT_ENTERED        4
#define  UNIT_NOT_PROVIDED      5
#define  INCORRECT_INPUT        6
#define  NUMBER_OF_ERRORS       6   

    // table of error messages
    // if your function never returns an
    // error -- you do not need to create this
    // table
    char * myErrorMessageTable[NUMBER_OF_ERRORS] =  
    {   "interrupted",
        "insufficient memory",
        "must be real",
        "The file path provided could not be found",
        "Units have not been provided at the top of the input file",
        "The inputs provided do not appear to exist in the provided file"
    };

    double Rad2Deg(double angle){
        return angle * 180.0 / M_PI;
    }

    double Deg2Rad(double angle){
        return angle * M_PI / 180.0;
    }

    // this code executes a check to get values from a csv and returning it as a string to MathCAD
    LRESULT  BoltICR(COMPLEXSCALAR* const Result,
        LPCCOMPLEXARRAY BoltList, LPCCOMPLEXSCALAR NumberBolts, LPCCOMPLEXSCALAR Fx, LPCCOMPLEXSCALAR Fy, LPCCOMPLEXSCALAR M)
    {
        if (NumberBolts->imag != 0.0 || Fx->imag != 0.0 || Fy->imag != 0.0 || M->imag != 0.0 || BoltList->hImag != NULL){
            return MUST_BE_REAL;
        }
        int numberOfBolts = NumberBolts->real;
        if (numberOfBolts == 0) {
            Result->real = 0;
            Result->imag = 0;
            return 0;
        }
        double fx = Fx->real;
        double fy = Fy->real;
        if ((fx == 0 && fy == 0) || numberOfBolts <= 1) {
            Result->real = numberOfBolts;
            Result->imag = 0;
            return 0;
        }
        double deltaMax = 0.34;
        int iterMax = 20000;
        double tolerance = 0.0001;
        double m = M->real;
        Bolt2D* bolts = NULL;
        for (int i = 0; i < numberOfBolts; i++){
            bolts = add_bolt(bolts, BoltList->hReal[0][i], BoltList->hReal[1][i], BoltList->hReal[2][i], BoltList->hReal[3][i], BoltList->hReal[4][i]);
        }
        if (bolts == NULL){
            Result->real = numberOfBolts;
            Result->imag = 0;
            return 0;
        }

        Vector3D* zVec = CreateVector(0, 0, 1);
        Vector3D* loadVector = CreateVector(fx, fy, 0);
        double Freal = GetVectorMagnitude(loadVector);
        Vector3D* Fu = GetUnitVector(loadVector);
        double eu = _abs64(m)/ Freal;
        if (eu <= 0.1){
            free_bolt_list(bolts);
            free(Fu);
            free(loadVector);
            free(zVec);
            Result->real = numberOfBolts;
            Result->imag = 0;
            return 0;
        }
        double forceAngle = Rad2Deg(GetAngleFromComponents(fx, fy));
        Vector3D* eVec = EccentricDirection(Deg2Rad(forceAngle));
        

        if (M > 0){
            Vector3D* oldZ = zVec;
            Vector3D* olde = eVec;
            zVec = MultiplyVector(-1, zVec);
            eVec = MultiplyVector(-1, eVec);
            free(oldZ);
            free(olde);
        }

        Vector3D* e = MultiplyVector(eu, eVec);
        double SumAb = 0;
        double IzBoltGroup = 0;
        double Xcen = 0;
        double Ycen = 0;
        BoltGroupCentroid(bolts, &Xcen, &Ycen);
        double Ru = bolts->Ab * bolts->Fnv;

        Bolt2D* list = bolts;
        while (list != NULL){
            list->X = list->X - Xcen;
            list->Y = list->Y - Ycen;
            double iXc = list->Ab * pow(list->X, 2);
            double iYc = list->Ab * pow(list->Y, 2);
            SumAb += list->Ab;
            IzBoltGroup += iXc + iYc;
            list = list->next;
        }

        Vector3D* tempVec = Cross(e, Fu);
        double tempMagnitude = GetVectorMagnitude(tempVec);
        free(tempVec);
        Vector3D* ICR = MultiplyVector((IzBoltGroup / (numberOfBolts * tempMagnitude * SumAb)), eVec);
        
        
        int iter = 0;
        while (iter < iterMax){
            Vector3D* posMax = CreateVector(0,0,0);
            list = bolts;
            //Get the maximum bolt position in the provided list
            while (list != NULL){
                Vector3D* boltPosition = CreateVector(list->X - ICR->X, list->Y - ICR->Y, 0);
                if (GetVectorMagnitude(posMax) < GetVectorMagnitude(boltPosition)){
                    Vector3D* tempVec = posMax;
                    posMax = boltPosition;
                    free(tempVec);
                }
                else {
                    free(boltPosition);
                }
                list = list->next;
            }
            
            list = bolts;
            //Get sum of forces and moments
            Vector3D* sumForces = CreateVector(0,0,0);
            double sumMoment = 0;
            while (list != NULL){
                Vector3D* boltPosition = CreateVector(list->X - ICR->X, list->Y - ICR->Y, 0);
                double boltDelta = GetVectorMagnitude(boltPosition) * deltaMax / GetVectorMagnitude(posMax);
                Vector3D* delta = MultiplyCrossUnitCombo(boltDelta, zVec, boltPosition);
                double boltF = Ru * pow(1 - pow(M_E, -10 * GetVectorMagnitude(delta)), 0.55);
                Vector3D* boltForce = MultiplyCrossUnitCombo(boltF, zVec, boltPosition);
                Vector3D* boltMoment = Cross(boltPosition, boltForce);
                sumForces->X += boltForce->X;
                sumForces->Y += boltForce->Y;
                sumForces->Z += boltForce->Z;
                sumMoment += GetVectorMagnitude(boltMoment);
                free(boltPosition);
                free(delta);
                free(boltForce);
                free(boltMoment);
                list = list->next;
            }

            Vector3D* PBoltGroup = CreateVector(sumForces->X, sumForces->Y, 0);
            Vector3D* tempCross1 = CreateVector(e->X - ICR->X, e->Y - ICR->Y, e->Z - ICR->Z);
            Vector3D* tempCross2 = Cross(tempCross1, Fu);
            Vector3D* scaledPBoltGroup = MultiplyVector(GetVectorMagnitude(tempCross2)/ sumMoment, PBoltGroup);
            free(tempCross1);
            free(tempCross2);
            Vector3D* errorCheck = CreateVector(scaledPBoltGroup->X + Fu->X, scaledPBoltGroup->Y + Fu->Y, scaledPBoltGroup->Z + Fu->Z);
            if (GetVectorMagnitude(errorCheck) <= tolerance){
                Result->real = GetVectorMagnitude(PBoltGroup) / Ru;
                Result->imag = 0;
                free_bolt_list(bolts);
                free(errorCheck);
                free(scaledPBoltGroup);
                free(PBoltGroup);
                free(sumForces);
                free(posMax);
                free(ICR);
                free(e);
                free(eVec);
                free(Fu);
                free(loadVector);
                free(zVec);
                return 0;
            }
            iter++;
            Vector3D* deltaTemp1 = Cross(zVec, errorCheck);
            Vector3D* deltaTemp2 = Cross(e, Fu);
            double multiplier = -1 * IzBoltGroup / (numberOfBolts * GetVectorMagnitude(deltaTemp2) * SumAb);
            Vector3D* ICRDelta = MultiplyVector(multiplier, deltaTemp1);
            ICR->X += ICRDelta->X;
            ICR->Y += ICRDelta->Y;
            ICR->Z += ICRDelta->Z;
            free(deltaTemp1);
            free(deltaTemp2);
            free(ICRDelta);
            free(errorCheck);
            free(scaledPBoltGroup);
            free(PBoltGroup);
            free(sumForces);
            free(posMax);
        }
        free_bolt_list(bolts);
        free(ICR);
        free(e);
        free(eVec);
        free(Fu);
        free(loadVector);
        free(zVec);
        Result->real = numberOfBolts;
        Result->imag = 0;
        return 0;
    }

    // fill out a FUNCTIONINFO structure with
    // the information needed for registering the
    // function with Mathcad
    FUNCTIONINFO ICR = 
    {
    // Name by which mathcad will recognize the function
    "BoltICR",        
    
    // description of "multiply" parameters to be used
    // by the Insert Function dialog box
    "boltList, numberOfBolts, Fx, Fy, M",   
    
    // description of the function for the Insert Function dialog box       
    "returns Bolt ICR of the given input",    
    
    // pointer to the executible code
    // i.e. code that should be executed
    // when a user types in "concreteBeamCalc(fc,fy,b,d,As)="
    (LPCFUNCTION)BoltICR,
        
    // GetValue(path,size,value) returns a string
    COMPLEX_SCALAR,
        
    // GetValue(path,size,value) takes in 3 inputs
    5,   
    
    // all 5 inputs are strings
    { COMPLEX_ARRAY, COMPLEX_SCALAR, COMPLEX_SCALAR, COMPLEX_SCALAR, COMPLEX_SCALAR}
    };
    

    // DLL entry point code
    // the _CRT_INIT function is needed
    // if you are using Microsoft's 32 bit compiler
 
    BOOL WINAPI _CRT_INIT(HINSTANCE hinstDLL, DWORD dwReason, LPVOID lpReserved);

    BOOL WINAPI  DllEntryPoint(HINSTANCE hDLL, DWORD dwReason, LPVOID lpReserved)
    {
        switch (dwReason)
        {
        case DLL_PROCESS_ATTACH:

            //
            // DLL is attaching to the address space of 
            // the current process.
            //
            if (!_CRT_INIT(hDLL, dwReason, lpReserved))
                return FALSE;


            // register the error message table
            // Note, that if your function never returns
            // an error -- you do not need to 
            // register an error message table
            if (CreateUserErrorMessageTable(
                hDLL, NUMBER_OF_ERRORS, myErrorMessageTable))
                // and if the errors register OK
                // go ahead and
                // register user function
                CreateUserFunction(hDLL, &ICR);
            break;


        case DLL_THREAD_ATTACH:
        case DLL_THREAD_DETACH:
        case DLL_PROCESS_DETACH:

            if (!_CRT_INIT(hDLL, dwReason, lpReserved))
                return FALSE;
            break;

        }
        return TRUE;
    }

#undef INTERRUPTED    
#undef INSUFFICIENT_MEMORY
#undef MUST_BE_REAL   
#undef NO_BOLT_ENTERED
#undef UNIT_NOT_PROVIDED
#undef INCORRECT_INPUT
#undef NUMBER_OF_ERRORS     
