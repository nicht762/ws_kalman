import pymap3
from serial import Serial
from pynmeagps import NMEAReader, NMEAMessage

ellipsoid = pymap3d.Ellipsoid()
# base
lat_0 = 55.760057829964516
lon_0 = 48.75896086562219
hgt_0 = 199.37

stream = Serial('/dev/ttyACM0', 115200, timeout=3)
nmr = NMEAReader(stream)

for (raw_data, parsed_data) in nmr:
    if parsed_data.msgID == 'GGA':
        #print(parsed_data)
        east, north, up = pymap3d.geodetic2enu(parsed_data.lat, parsed_data.lon, parsed_data.alt,
                                               lat_0, lon_0, hgt_0,
                                               ell=ellipsoid, deg=True)
        print(east, north, up)
