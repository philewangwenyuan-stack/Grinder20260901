plugins {
    id("com.android.library") version "7.4.2"
    id("org.jetbrains.kotlin.android") version "1.8.0"
}

android {
    namespace = "sl_link"
    compileSdk = 34

    defaultConfig {
        minSdk = 24
        targetSdk = 34

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("consumer-rules.pro")
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    
    kotlinOptions {
        jvmTarget = "1.8"
    }
    
    // Source directories
    sourceSets {
        getByName("main") {
            java.srcDirs(
                "src/frame",       // 帧处理（手写）
                "src/message_gen", // 消息（protobuf 生成）
                "examples"
            )
        }
    }
    
    // Exclude duplicate proto files from protobuf-java and protobuf-javalite
    packagingOptions {
        resources.excludes.add("google/protobuf/*.proto")
        resources.excludes.add("META-INF/*.proto")
    }
}

dependencies {
    // Protobuf runtime (code is pre-generated)
    // Use full protobuf-kotlin (not lite) because custom options (unit, scale)
    // require DescriptorProtos which is not in javalite
    implementation("com.google.protobuf:protobuf-kotlin:3.24.0")
    
    // Kotlin
    implementation("org.jetbrains.kotlin:kotlin-stdlib:1.9.0")
    
    // Testing
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
}
