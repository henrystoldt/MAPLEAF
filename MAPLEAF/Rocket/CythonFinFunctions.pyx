# distutils: language = c++

''' Cython functions to speed up the `MAPLEAF.Rocket.FinSet` aerodynamic model '''

from MAPLEAF.Motion.CythonVector cimport Vector
from libc.math cimport asin, abs
from libcpp.vector cimport vector
from math import isnan

cpdef double getFinSliceAngleOfAttack(double finSpanPosition, Vector airVelRelativeToFin, Vector unitSpanTangentialAirVelocity, Vector finNormal, Vector spanDirection, double stallAngle):
    ''' Finds the AOA of the present a fin slice. Limits AOA to a magnitude of stallAngle '''
    # We need to find the angle of attack of each slice of the fin. When roll velocity is zero, those are all the same
    cdef Vector finVelocity = airVelRelativeToFin + unitSpanTangentialAirVelocity*finSpanPosition  #In the local frame
    cdef double finVelMag = finVelocity.length()
    
    cdef double FSAOA = 0.0
    if finVelMag >= 0.1:
        # This limits us to fin angles of attack of less than 90 degrees (first and fourth quadrants)
        body = finNormal*finVelocity * (1/finVelMag)
        if abs(body - 1) < 1e-14:
            body = 1.0
        FSAOA = asin(body)

    # Limit angle of attack magnitude to stall angle
    if abs(FSAOA) > stallAngle:
        FSAOA *= abs(stallAngle/FSAOA)

    return FSAOA

cpdef double getFinSliceForce_Supersonic(double K1, double K2, double K3, double sliceAOA, double sliceArea):
    ''' Computes normal force produced by a supersonic fin surface slice, excluding dynamic pressure. '''
    # TODO: Include mach-cone correction (third-order term)
    # TODO: Include term that changes depending on whether behind shock or not
    # Supersonic correlations, Eq. 3.41-3.44 (Niskanen)
    # Using third-order Busemann expansion, was also used by Barrowman.
    # Neglecting mach-cone effects & shock effects
    cdef double topCp = sliceAOA*( K1 + K2*sliceAOA + K3*sliceAOA**2 )
    cdef double bottomCp = -sliceAOA*( K1 + K2*(-sliceAOA) + K3*(-sliceAOA)**2 )
    return (topCp - bottomCp) * sliceArea

cpdef getSubsonicFinNormalForce(Vector airVelRelativeToFin, Vector unitSpanTangentialAirVelocity, Vector finNormal, Vector spanDirection, double spanwiseCP, double CnAlpha, fin):
    cdef double normalForceMagnitude = 0.0
    cdef int nSlices = len(fin.spanSliceAreas)
    cdef int i = 0
    cdef double FSAOA

    cdef double stallAngle = fin.stallAngle
    cdef double moment = 0
    cdef vector[double] sliceAreas = fin.spanSliceAreas
    cdef vector[double] sliceRadii = fin.spanSliceRadii
    while i < nSlices:
        FSAOA = getFinSliceAngleOfAttack(sliceRadii[i], airVelRelativeToFin, unitSpanTangentialAirVelocity, finNormal, spanDirection, stallAngle)
        if isnan(FSAOA):
            for x in (sliceRadii[i], airVelRelativeToFin, unitSpanTangentialAirVelocity, finNormal, spanDirection, stallAngle):
                print(x)
        sliceForce = CnAlpha * FSAOA * sliceAreas[i]
        normalForceMagnitude += sliceForce

        # In addition to total normal force (applied at the CP), calculate the moment about the CP (about +Z direction)
        moment += sliceForce * (spanwiseCP - sliceRadii[i])
        i += 1
    
    return normalForceMagnitude, moment

cpdef getSupersonicFinNormalForce(Vector airVelRelativeToFin, Vector unitSpanTangentialAirVelocity, Vector finNormal, Vector spanDirection, double spanwiseCP, double K1, double K2, double K3, fin):
    cdef double normalForceMagnitude = 0.0
    cdef int nSlices = len(fin.spanSliceAreas)
    cdef int i = 0
    cdef double FSAOA
    
    cdef double stallAngle = fin.stallAngle
    cdef double moment = 0
    cdef vector[double] sliceAreas = fin.spanSliceAreas
    cdef vector[double] sliceRadii = fin.spanSliceRadii
    while i < nSlices:       
        FSAOA = getFinSliceAngleOfAttack(sliceRadii[i], airVelRelativeToFin, unitSpanTangentialAirVelocity, finNormal, spanDirection, stallAngle)
        sliceForce = getFinSliceForce_Supersonic(K1, K2, K3, FSAOA, sliceAreas[i])
        normalForceMagnitude += sliceForce
        
        # In addition to total normal force (applied at the CP), calculate the moment about the CP (about +Z direction)
        moment += sliceForce * (spanwiseCP - sliceRadii[i])
        i += 1
    
    return normalForceMagnitude, moment