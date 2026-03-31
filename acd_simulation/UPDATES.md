Notes & Observations
- [Week of Jan 26] ```simulate_multiple_objects.py```: the use of ```arcsin``` causes spatial aliasing which leads to messed up measured AoA when it exceeds -90/90 degrees
- [Week of Feb 02]  ```simulate_multiple_objects.py```: added a 3rd fft for spatial + changed aoa compute logic, increased the number of antennas to 4
- [Week of Feb 09]  ```multi_cartesian.py```: the aoa isn't fixed yet (TODO: need to migrate over)
- [3/31] ```processing.py``` has basic aoa estimation (phase diff ->  angle using array geometry). Run ```stream_data_cs_team.py``` with ```IS_SNAPSHOT_MODE = False``` to test the 0, 45, -45 degrees cases