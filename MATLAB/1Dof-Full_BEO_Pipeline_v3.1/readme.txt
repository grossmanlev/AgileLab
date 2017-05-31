one_dof_full_BEO_pipeline.m is the main file, see inside for details;
this file runs the modelnet 10 experiment.

singleObjectRotClassifyComplete.m is the workhorse file. Given a (learned) shared subspace and learned
means/covariences for each class, it does 1dof (about z axis) rotation estimation, classification, and completion
for an arbitrary query object (voxel format, already aligned, -1 is unkown, 1 is filled, 0 is empty). This file is not
specific to modelNet10 and works with any input object (assuming, of course, that a suitable subspace and class model is
also provided).

if you do not have the pretrained basis using VBPCA, or want to retrain, use findBasis_vbpca
which operates on a dir containing voxelized object mat files

This code is a bit of a work in progress, there is currently not a totally general version that does end-end training with a novel
dataset of mesh objects in 1 shot.
To do this:

- The mesh (training) objects must first be voxelized and aligned (not included in this package)

- VBPCA must be performed (see findBasis_vbpca.m). The included file is mostly general, but some things like class names ect
   at the top of the file msut be changed

- Training most occur (find shared subspace, learn class models) one_dof_full_BEO_pipeline.m contains the code for this, but
  a small bit (at the top) must be changed for new data (things like class names)

- At test time, the partially observed object must be aligned and converted to a voxel object. This entails aligning
  translationally and aligning x and y rotation (i.e. it should be oriented upwards). Additonally the voxel size (resolution)
  must be appropriatly set for that class of object. There are ways this may be addressed automatically,
  but currently it is treaded like segmentation - outside the scope of our work.

- At test time, once the above step is done, the partialy observed voxel object should be fed into singleObjectRotClassifyComplete_1dof.m
  This file is general and should not need to be modified.
  
  There is also a handy tool to visualize a completed (or training) voxel object. Simply call visualizeObject(object) where object is
  a 3D binary array representing a voxel object (0 unfilled, 1 filled),
  
