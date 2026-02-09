Notes & Observations
- [Week of Jan 26] ```simulate_multiple_objects.py```: the use of ```arcsin``` causes spatial aliasing which leads to messed up measured AoA when it exceeds -90/90 degrees
- [Week of Feb 02]  ```simulate_multiple_objects.py```: added a 3rd fft for spatial + changed aoa compute logic, increased the number of antennas to 4
- [Week of Feb 09] the aoa isn't fixed in ```multi_cartesian.py``` (TODO: need to migrate over)