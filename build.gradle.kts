plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
}

ihmc {
   group = "us.ihmc"
   version = "17-0.11.6"
   vcsUrl = "https://github.com/ihmcrobotics/mecano"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid:0.19.1")
   api("us.ihmc:euclid-frame:0.19.1")
   api("us.ihmc:euclid-geometry:0.19.1")
}

testDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-simple:0.39")

   api("us.ihmc:log-tools:0.6.3")

   var javaFXVersion = "17.0.2"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
   api(ihmc.javaFXModule("fxml", javaFXVersion))
}

graphvizDependencies {
   api(ihmc.sourceSetProject("main"))

   api("guru.nidi:graphviz-java:0.5.4")
}

yovariablesDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-yovariables:0.9.16")
}
