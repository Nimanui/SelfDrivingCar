# SelfDrivingCar
GitHub repo for collaboration with Lab 1B CS437

# Run on course
for Picar-X
`python3 navigate_path.py`
for Picar 4WD
`python3 navigate_path4wd.py`

There is a sample file of the sort of output you can expect to see with the car running through a course "output12.txt".

## Test
- Test advanced mapping in your computer with mocks:

`python -m unittest test/test_advanced_mapping.py`


- Test the A* path:

`python -m unittest test/test_path_finder.py`


## Path visual
Following is example of the car finding the path avoiding obstacles:

<img src="./car_footprint_figure.png" width="800px"/>
