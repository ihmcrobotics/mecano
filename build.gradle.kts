plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
}

ihmc {
   group = "us.ihmc"
   version = "0.7.4"
   vcsUrl = "https://github.com/ihmcrobotics/mecano"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid:0.16.2")
   api("us.ihmc:euclid-frame:0.16.2")
   api("us.ihmc:euclid-geometry:0.16.2")
}

testDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-simple:0.39")
}

graphvizDependencies {
   api(ihmc.sourceSetProject("main"))

   api("guru.nidi:graphviz-java:0.5.4")
}

yovariablesDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-yovariables:0.9.8")
}