# Trajectory generation and control of 2-dimensional quadcopter in cluttered environments

<!---
### Abstract
`Iterative Closest Point` (ICP) is one of the widely used algorithms in aligning three dimensional models given an initial guess of the rigid body transformation required. In this algorithm, one point cloud, the reference, is kept fixed, while the other one, the source, is transformed to best match the reference. The algorithm iteratively revises the transformation (combination of translation and rotation) needed to minimize an error metric, usually a distance from the source to the reference point cloud, such as the sum of squared differences between the coordinates of the matched pairs. In each step various parts of the algorithm can be parallelized to improve the performance. At first, we plan to use OpenMP to parallelize the ICP algorithm. If time permits, we will try to write CUDA code and use GPU to improve the performance further.

### Compiling
```bash
mkdir -p build
cd build
cmake ..
make
```
This will create an executable `icp` inside build directory.

### Usage
```bash
./icp <config_file>
```

config file is of XML format, following is a sample:
```xml
<icp>
    <scene>../scenes/cone.txt</scene>
    <target>../scenes/cone.txt</target>
    <numThreads>16</numThreads>
    <numIterations>5</numIterations>
    <errorFile>error_history.csv</errorFile>
    <outDir>output</outDir>
    <saveInterval>10</saveInterval>

    <initialTransform>
        <translate>
            <x>80.0</x>
            <y>-22.0</y>
            <z>100.0</z>
        </translate>
        <rotate>
            <x>-0.5</x>
            <y>0.6</y>
            <z>0.8</z>
        </rotate>
    </initialTransform>
</icp>
```

-->
