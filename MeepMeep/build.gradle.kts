plugins {
    kotlin("jvm") version "1.6.10"
}

java {
    sourceCompatibility = JavaVersion.VERSION_11
    targetCompatibility = JavaVersion.VERSION_11
}

repositories {
    maven (url = "https://jitpack.io")
}

dependencies {
    implementation("com.github.NoahBres:MeepMeep:1.0.6")
    implementation("org.jetbrains.kotlin:kotlin-script-runtime:1.6.10")
}
