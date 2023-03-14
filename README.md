# mini_cuboid
Lab 1 implements the `LinearCharacteristics` class and `IIR_filter` class for PT1 filter 
of the type 
```math
y_k = -a_0y_{k-1}+b_0u.
```
Additionally the sensors are calibrated with the 
```python
LinearCharacteristics::LinearCharacteristics(float xmin,float xmax, float ymin, float ymax)
``` 
constructor and evaluated with the
```python
LinearCharacteristics::evaluate(float x)
```
method. </br></br>
The resulting filtered IMU reading can be seen in the following figure:
![output2](https://user-images.githubusercontent.com/119895438/221898008-8c2dd77d-ea21-4a51-b3de-1a76e76e0e43.png)
