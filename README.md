# ![Mecano](logo/Mecano.png)
![mecano](https://maven-badges.herokuapp.com/maven-central/us.ihmc/mecano/badge.svg?style=plastic)
![buildstatus](https://github.com/ihmcrobotics/mecano/actions/workflows/gradle.yml/badge.svg)


## Minutiae

### Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Test and documentation coverage
We have put our best effort into documenting and testing the entire library. 

### Licensing
This library is under the license Apache 2.0. Consult the license file for more information.

### Compatibility
This library is compatible with Java 17+ and Gradle 7.5+.

### Dependency
This library main source depends on the matrix library EJML [here](http://ejml.org/) and Euclid [here](https://github.com/ihmcrobotics/euclid).

## What is Mecano?
Mecano provides algorithms for rigid-body dynamics and a framework to define multi-body systems articulated with joints.
The library is born from the need of having a base library for whole-body control that is well tested, flexible, and meant for real-time environment where garbage generation is not allowed.
This library is meant to be the base of the control framework that is still in development at IHMC and will be released progressively in the near future.

## How does Mecano work?
TODO

## Who would use Mecano?
Any control engineer wanting to implement whole-body controller or implement a simulator is susceptible to use this library as the base to simplify the math involved.

## What is the goal of Mecano?
The goal for Mecano is to become the most flexible and easy to use, so it results in great increase in development productivity. 
It is unclear yet the extent of this library in the future.

## How can I contribute to Mecano?
Please read [CONTRIBUTING.md](https://github.com/ihmcrobotics/mecano/blob/develop/CONTRIBUTING.md).

## Content
This library includes the following:
- The definition of spatial vectors used to represent 6D velocity, i.e. Twist, 6D acceleration, i.e. SpatialAcceleration, etc.
- The definition of joints, e.g. RevoluteJoint, SixDoFJoint, and rigid-bodies, i.e. RigidBody, to create multi-body systems.
- Algorithms to compute:
	- Center of mass position and velocity: CenterOfMassCalculator, CenterOfMassJacobian.
	- Rigid-body spatial acceleration: SpatialAccelerationCalculator.
	- Centroidal momentum matrix: CentroidalMomentumCalculator and CentroidalMomentumRateCalculator.
	- Geometric Jacobian: GeometricJacobianCalculator.
	- Inverse and forward dynamics: InverseDynamicsCalculator, ForwardDynamicsCalculator.
	- The mass matrix: CompositeRigidBodyMassMatrixCalculator.
	- Response to external disturbance (wrench or impulse): MultiBodyResponseCalculator.

## Using Mecano from .jar releases with Maven/Gradle
The releases .jars for Mecano are hosted on Maven repository.
You can browse the IHMC release packages at https://mvnrepository.com/artifact/us.ihmc.

At a minimum, you will need to have the following repository declared in your build script to use the Mecano .jars:

```gradle
repositories {
   mavenCentral()
}
```

Here is an example for adding the dependency to Mecano using your build script:

```gradle
dependencies {
   compile group: "us.ihmc", name: "mecano", version: "x.x"
}
```
[ ![mecano](https://maven-badges.herokuapp.com/maven-central/us.ihmc/mecano/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/mecano)
