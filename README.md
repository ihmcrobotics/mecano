# ![Mecano](logo/Mecano.png)
[ ![mecano](https://api.bintray.com/packages/ihmcrobotics/maven-release/mecano/images/download.svg) ](https://bintray.com/ihmcrobotics/maven-release/mecano/_latestVersion)
[ ![buildstatus](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-MECANO)](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-MECANO)

## Minutiae

### Tested Platforms
We test all of our software on OS X 10.11 El Capitan, Windows 7/8/10, and Ubuntu 16.04 LTS Desktop and Server.

### Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Test and documentation coverage
We have put our best effort into documenting and testing the entire library. 

### Licensing
This library is under the license Apache 2.0. Consult the license file for more information.

### Compatibility
This library is compatible with Java 8+ and Gradle 4.10+.

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
	- (Under development) Impulse propagation: MultiBodyImpulseCalculator.

## Using Mecano from .jar releases with Maven/Gradle
The releases .jars for Mecano are hosted on Bintray.
You can browse the IHMC release packages at https://bintray.com/ihmcrobotics/maven-release.
Instructions for adding the Maven repository and identifying the artifacts can also be found on Bintray for each package.

At a minimum, you will need to have the following repository declared in your build script to use the Mecano .jars:

```gradle
repositories {
   maven {
      url  "https://dl.bintray.com/ihmcrobotics/maven-release" // IHMC Code releases
   }

   /* You will also need to add either jcenter() or mavenCentral() or both, depending on your preference */
}
```

Here is an example for adding the dependency to Mecano using your build script:

```gradle
dependencies {
   compile group: "us.ihmc", name: "mecano", version: "x.x"
}
```
[ ![mecano](https://api.bintray.com/packages/ihmcrobotics/maven-release/mecano/images/download.svg) ](https://bintray.com/ihmcrobotics/maven-release/mecano/_latestVersion)
