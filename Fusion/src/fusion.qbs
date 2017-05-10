import qbs 1.0

Project {
    name: "fusion"
    minimumQbsVersion: "1.6"

    StaticLibrary {
        name: "sensors"

        Depends { name: "cpp" }

        cpp.defines: [
            "NON_MATLAB_PARSING",
            "MAX_EXT_API_CONNECTIONS=255"
        ]

        cpp.cxxLanguageVersion: "c++14"
        cpp.includePaths: [ "sensors/" ]

        Export {
            Depends { name: "cpp" }
            cpp.dynamicLibraries: [ "pthread" ]
        }

        files: [
            "sensors/crbuffer.h",
            "sensors/extApi.cpp",
            "sensors/extApi.h",
            "sensors/extApiPlatform.cpp",
            "sensors/extApiPlatform.h",
            "sensors/fetcher.cpp",
            "sensors/fetcher.h",
            "sensors/vrep_client.cpp",
            "sensors/vrep_client.h",
        ]
    }

    StaticLibrary {
        name: "view"

        Depends { name: "cpp" }

        cpp.cxxLanguageVersion: "c++14"
        cpp.includePaths: [ "view/", "." ]

        Export {
            Depends { name: "cpp" }
            Depends { name: "sensors" }
            cpp.dynamicLibraries: [ "glfw", "png", "GLEW", "GL", "GLU" ]
        }

        files: [
            "view/texture.h",
            "view/vbuffer.cpp",
            "view/vbuffer.h",
            "view/view.h",
            "view/2d_shader.frag",
            "view/2d_shader.vert",
            "view/font.cpp",
            "view/font.h",
            "view/image.h",
            "view/shader.cpp",
            "view/shader.h",
            "view/view_debug.cpp",
            "view/view_debug.h",
            "view/view_window.cpp",
            "view/view_window.h",
        ]
    }

    CppApplication {
        name: "fusion"

        Depends { name: "view" }

        cpp.includePaths: [ "." ]
        cpp.cxxLanguageVersion: "c++14"

        files: [ "main.cpp" ]
    }
}
