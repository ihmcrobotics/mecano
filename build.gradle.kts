plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   group = "us.ihmc"
   version = "17-0.19.2"
   vcsUrl = "https://github.com/ihmcrobotics/mecano"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid:0.22.3")
   api("us.ihmc:euclid-frame:0.22.3")
   api("us.ihmc:euclid-geometry:0.22.3")
}

testDependencies {
   api(ihmc.sourceSetProject("yovariables"))

   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-simple:0.39")

   api("us.ihmc:log-tools:0.6.5")

   var javaFXVersion = "17.0.8"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
   api(ihmc.javaFXModule("fxml", javaFXVersion))
}

graphvizDependencies {
   api(ihmc.sourceSetProject("main"))

   api("guru.nidi:graphviz-java-all-j2v8:0.18.1")
}

yovariablesDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-yovariables:0.13.6")
}

yovariablesFiltersDependencies {
   api(ihmc.sourceSetProject("yovariables"))

   api("us.ihmc:ihmc-yovariables-filters:0.13.6")
}
