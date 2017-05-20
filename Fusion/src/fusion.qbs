import qbs 1.0
import qbs.Probes

Project {
    name: "fusion"
    minimumQbsVersion: "1.6"

    Product {
        name: "pcl"
        Export {
            Depends { name: "cpp" }
            cpp.systemIncludePaths: [
                "/usr/include/pcl-1.8/",
                "/usr/include/eigen3/"
            ]
            cpp.dynamicLibraries: [
                "pcl_common",
                "pcl_octree",
                "pcl_search",
                "pcl_surface",
                "pcl_kdtree",
                "flann_cpp"
            ]
        }
    }

    StaticLibrary {
        name: "sensors"

        Depends { name: "cpp" }
        Depends { name: "pcl" }

        cpp.defines: [
            "NON_MATLAB_PARSING",
            "MAX_EXT_API_CONNECTIONS=255"
        ]

        cpp.cxxLanguageVersion: "c++14"
        cpp.useCxxPrecompiledHeader: true
        cpp.includePaths: [ "sensors/" ]

        Export {
            Depends { name: "pcl" }
            Depends { name: "cpp" }
            cpp.dynamicLibraries: [ "pthread" ]
        }

        Group {
            name: "pch"
            files: ["sensors/pch.h"]
            fileTags: ["cpp_pch_src"]
        }

        files: [
            "sensors/crbuffer.h",
            "sensors/extApi.cpp",
            "sensors/extApi.h",
            "sensors/extApiPlatform.cpp",
            "sensors/extApiPlatform.h",
            "sensors/mesh.cpp",
            "sensors/mesh.h",
            "sensors/mesh_types.cpp",
            "sensors/point_cloud.cpp",
            "sensors/point_cloud.h",
            "sensors/sensors.cpp",
            "sensors/sensors.h",
            "sensors/vrep_client.cpp",
            "sensors/vrep_client.h",
        ]
    }

    StaticLibrary {
        name: "view"

        Depends { name: "cpp" }
        Depends { name: "sensors" }

        cpp.cxxLanguageVersion: "c++14"
        cpp.includePaths: [ "view/", "." ]

        Export {
            Depends { name: "sensors" }
            Depends { name: "cpp" }
            cpp.dynamicLibraries: [ "glfw", "png", "GLEW", "GL", "GLU" ]
        }

        files: [
            "view/3d_shader.frag",
            "view/3d_shader.vert",
            "view/main_view.cpp",
            "view/main_view.h",
            "view/point_shader.frag",
            "view/point_shader.vert",
            "view/renderer.cpp",
            "view/renderer.h",
            "view/shaders.cpp",
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

