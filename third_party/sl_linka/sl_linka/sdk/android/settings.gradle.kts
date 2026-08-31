// SL-LinkA Android SDK
// 
// 使用方式：在主项目的 settings.gradle.kts 中添加：
//   include(":sl_link")
//   project(":sl_link").projectDir = file("path/to/sl-link/sdk/android")

pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "sl_link"
