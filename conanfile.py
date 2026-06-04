from conan import ConanFile


class GbbcppEmuDependencies(ConanFile):
    name = "gbbcpp_emu"
    version = "0.1.0"
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeDeps", "CMakeToolchain"

    default_options = {
        "sdl/*:alsa": False,
        "sdl/*:pulse": False,
        "sdl/*:wayland": False,
        "sdl/*:opengl": False,
        "sdl/*:opengles": False,
        "sdl/*:vulkan": False,
        "sdl/*:libunwind": False,
        "sdl/*:hidapi": False,
        "sdl/*:xcursor": False,
        "sdl/*:xinerama": False,
        "sdl/*:xinput": False,
        "sdl/*:xrandr": False,
        "sdl/*:xscrnsaver": False,
        "sdl/*:xshape": False,
        "sdl/*:xvm": False,
    }

    def requirements(self):
        self.requires("boost/1.86.0")
        self.requires("gtest/1.15.0")
        self.requires("portable-file-dialogs/0.1.0")
        self.requires("sdl/2.28.3")
        self.requires("sdl_ttf/2.22.0")
