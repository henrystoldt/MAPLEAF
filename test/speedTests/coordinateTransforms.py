from timeit import timeit
import pyproj

nTests = 100
print("Each operation performed {} times".format(nTests))


setupTransforms = \
'''
from MAPLEAF.ENV import WGS84Earth, SphericalEarth
import pyproj

wgs = WGS84Earth()
sphere = SphericalEarth()

ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')

lat, lon, h = 81.136427, -34.544050, 500
x, y, z = 812209.7821296552, -559136.2029726238, 6280831.753924148
'''


print("\n\n ----- Custom ----- ")

print("Geodetic -> Cartesian")
print(timeit("x, y, z = wgs.geodeticToCartesian(lat, lon, h)", setup=setupTransforms, number=nTests))

print("Cartesian -> Geodetic")
print(timeit("lat, lon, h = wgs.cartesianToGeodetic(x, y, z)", setup=setupTransforms, number=nTests))



print("\n\n ----- PyProj ----- ")

print("Geodetic -> Cartesian")
print(timeit("x, y, z = pyproj.transform(lla, ecef, lon, lat, h, radians=False)", setup=setupTransforms, number=nTests))

print("Cartesian -> Geodetic")
print(timeit("lat, lon, h = pyproj.transform(ecef, lla, x, y, z, radians=False)", setup=setupTransforms, number=nTests))