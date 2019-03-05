Various files related to the IMU's. However, the main artefact here is a procedure for for finding the orientation of an IMU. 
The IMU is ususally mounted by an offset, as opposed to what is considered ideal. This offset is described by three Euler angles. By finding these three euler angles and forming a rotation matrix from the offset frame, to the ideal frame, all IMU data can be rotated such that it appears in an ideal frame. This was tested in practice and proved to work really well. It is especially useful when the IMU cannot be mounted in its desired orientation due to mechanical constraints. 

Run FindIMUpos for a sample. 

By Love Palm and David Wall

