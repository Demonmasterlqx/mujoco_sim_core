#include <mujoco_sim/sensor/offscreen_renderer.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <dlfcn.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/osmesa.h>
#include <mujoco/mujoco.h>

#include "internal.hpp"

namespace mujoco_sim::sensor {
namespace {

enum class PlatformFamily { kEgl, kOsMesa };

// MuJoCo's OpenGL dispatch table is process-global after the first context.
std::optional<PlatformFamily> g_mujoco_platform;
void* g_mujoco_platform_library = nullptr;
std::mutex g_mujoco_platform_mutex;

PlatformFamily FamilyOf(OffscreenBackend backend) {
  return backend == OffscreenBackend::kOsMesa ? PlatformFamily::kOsMesa
                                               : PlatformFamily::kEgl;
}

bool ResolveBackendPreference(BackendPreference requested,
                              BackendPreference* effective,
                              std::string* error_message) {
  if (!g_mujoco_platform.has_value()) {
    *effective = requested;
    return true;
  }
  const BackendPreference locked =
      *g_mujoco_platform == PlatformFamily::kEgl
          ? BackendPreference::kEglOnly
          : BackendPreference::kOsMesaOnly;
  if (requested != BackendPreference::kAutomatic && requested != locked) {
    return detail::Fail(
        "MuJoCo OpenGL dispatch is already initialized for the other "
        "offscreen platform in this process",
        error_message);
  }
  *effective = locked;
  return true;
}

template <typename Function>
bool LoadSymbol(void* library, const char* name, Function* function,
                std::string* error_message) {
  dlerror();
  void* symbol = dlsym(library, name);
  const char* error = dlerror();
  if (symbol == nullptr || error != nullptr) {
    *error_message = std::string("could not load ") + name + ": " +
                     (error == nullptr ? "symbol not found" : error);
    return false;
  }
  *function = reinterpret_cast<Function>(symbol);
  return true;
}

void* OpenPlatformLibrary(std::string_view versioned_name,
                          std::string_view unversioned_name,
                          std::string* error_message) {
  void* library = dlopen(std::string(versioned_name).c_str(),
                         RTLD_NOW | RTLD_GLOBAL);
  if (library == nullptr) {
    library = dlopen(std::string(unversioned_name).c_str(),
                     RTLD_NOW | RTLD_GLOBAL);
  }
  if (library == nullptr) {
    const char* error = dlerror();
    *error_message = error == nullptr ? "platform library not found" : error;
  }
  return library;
}

class NativeContext {
 public:
  NativeContext() = default;
  ~NativeContext() { Destroy(); }

  NativeContext(const NativeContext&) = delete;
  NativeContext& operator=(const NativeContext&) = delete;

  bool Initialize(std::uint32_t width, std::uint32_t height,
                  BackendPreference preference, std::string* error_message) {
    std::string egl_error;
    if (preference != BackendPreference::kOsMesaOnly &&
        TryEgl(&egl_error)) {
      return true;
    }
    if (preference == BackendPreference::kEglOnly) {
      return detail::Fail(egl_error, error_message);
    }

    std::string osmesa_error;
    if (TryOsMesa(width, height, &osmesa_error)) {
      return true;
    }
    if (preference == BackendPreference::kOsMesaOnly) {
      return detail::Fail(osmesa_error, error_message);
    }
    return detail::Fail("EGL initialization failed (" + egl_error +
                            "); OSMesa fallback failed (" + osmesa_error +
                            ")",
                        error_message);
  }

  bool MakeCurrent(std::string* error_message) {
    switch (backend_) {
      case OffscreenBackend::kEglSurfaceless:
        if (egl_api_.make_current(egl_display_, EGL_NO_SURFACE,
                                  EGL_NO_SURFACE, egl_context_) == EGL_TRUE) {
          return true;
        }
        return detail::Fail(EglErrorString("eglMakeCurrent"), error_message);
      case OffscreenBackend::kEglPbuffer:
        if (egl_api_.make_current(egl_display_, egl_surface_, egl_surface_,
                                  egl_context_) == EGL_TRUE) {
          return true;
        }
        return detail::Fail(EglErrorString("eglMakeCurrent"), error_message);
      case OffscreenBackend::kOsMesa:
        if (osmesa_context_ != nullptr && !osmesa_buffer_.empty() &&
            osmesa_api_.make_current(osmesa_context_, osmesa_buffer_.data(),
                                     GL_UNSIGNED_BYTE, osmesa_width_,
                                     osmesa_height_) == GL_TRUE) {
          return true;
        }
        return detail::Fail("OSMesaMakeCurrent failed", error_message);
    }
    return detail::Fail("offscreen backend is not initialized",
                        error_message);
  }

  OffscreenBackend backend() const noexcept { return backend_; }

  void RetainDispatchLibraryForProcess(void** retained_library) noexcept {
    if (retained_library == nullptr || *retained_library != nullptr) {
      return;
    }
    void*& library = backend_ == OffscreenBackend::kOsMesa
                         ? osmesa_library_
                         : egl_library_;
    *retained_library = library;
    library = nullptr;
  }

 private:
  struct EglApi {
    decltype(&eglGetError) get_error = nullptr;
    decltype(&eglGetProcAddress) get_proc_address = nullptr;
    decltype(&eglInitialize) initialize = nullptr;
    decltype(&eglBindAPI) bind_api = nullptr;
    decltype(&eglChooseConfig) choose_config = nullptr;
    decltype(&eglCreateContext) create_context = nullptr;
    decltype(&eglMakeCurrent) make_current = nullptr;
    decltype(&eglCreatePbufferSurface) create_pbuffer_surface = nullptr;
    decltype(&eglDestroySurface) destroy_surface = nullptr;
    decltype(&eglDestroyContext) destroy_context = nullptr;
    decltype(&eglTerminate) terminate = nullptr;
  };

  struct OsMesaApi {
    decltype(&OSMesaCreateContextExt) create_context = nullptr;
    decltype(&OSMesaMakeCurrent) make_current = nullptr;
    decltype(&OSMesaPixelStore) pixel_store = nullptr;
    decltype(&OSMesaDestroyContext) destroy_context = nullptr;
  };

  std::string EglErrorString(std::string_view operation) const {
    std::ostringstream output;
    output << operation << " failed with EGL error 0x" << std::hex
           << static_cast<unsigned int>(
                  egl_api_.get_error == nullptr ? 0 : egl_api_.get_error());
    return output.str();
  }

  bool LoadEgl(std::string* error_message) {
    egl_library_ =
        OpenPlatformLibrary("libEGL.so.1", "libEGL.so", error_message);
    if (egl_library_ == nullptr) {
      return false;
    }
    if (!LoadSymbol(egl_library_, "eglGetError", &egl_api_.get_error,
                    error_message) ||
        !LoadSymbol(egl_library_, "eglGetProcAddress",
                    &egl_api_.get_proc_address, error_message) ||
        !LoadSymbol(egl_library_, "eglInitialize", &egl_api_.initialize,
                    error_message) ||
        !LoadSymbol(egl_library_, "eglBindAPI", &egl_api_.bind_api,
                    error_message) ||
        !LoadSymbol(egl_library_, "eglChooseConfig", &egl_api_.choose_config,
                    error_message) ||
        !LoadSymbol(egl_library_, "eglCreateContext",
                    &egl_api_.create_context, error_message) ||
        !LoadSymbol(egl_library_, "eglMakeCurrent", &egl_api_.make_current,
                    error_message) ||
        !LoadSymbol(egl_library_, "eglCreatePbufferSurface",
                    &egl_api_.create_pbuffer_surface, error_message) ||
        !LoadSymbol(egl_library_, "eglDestroySurface",
                    &egl_api_.destroy_surface, error_message) ||
        !LoadSymbol(egl_library_, "eglDestroyContext",
                    &egl_api_.destroy_context, error_message) ||
        !LoadSymbol(egl_library_, "eglTerminate", &egl_api_.terminate,
                    error_message)) {
      DestroyEgl();
      return false;
    }
    return true;
  }

  bool LoadOsMesa(std::string* error_message) {
    osmesa_library_ = OpenPlatformLibrary("libOSMesa.so.8", "libOSMesa.so",
                                          error_message);
    if (osmesa_library_ == nullptr) {
      return false;
    }
    if (!LoadSymbol(osmesa_library_, "OSMesaCreateContextExt",
                    &osmesa_api_.create_context, error_message) ||
        !LoadSymbol(osmesa_library_, "OSMesaMakeCurrent",
                    &osmesa_api_.make_current, error_message) ||
        !LoadSymbol(osmesa_library_, "OSMesaPixelStore",
                    &osmesa_api_.pixel_store, error_message) ||
        !LoadSymbol(osmesa_library_, "OSMesaDestroyContext",
                    &osmesa_api_.destroy_context, error_message)) {
      DestroyOsMesa();
      return false;
    }
    return true;
  }

  bool TryEgl(std::string* error_message) {
    if (!LoadEgl(error_message)) {
      return false;
    }
    auto get_platform_display =
        reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
            egl_api_.get_proc_address("eglGetPlatformDisplayEXT"));
    if (get_platform_display == nullptr) {
      *error_message = "EGL_EXT_platform_base is unavailable";
      return false;
    }

    egl_display_ = get_platform_display(EGL_PLATFORM_SURFACELESS_MESA,
                                        EGL_DEFAULT_DISPLAY, nullptr);
    if (egl_display_ == EGL_NO_DISPLAY) {
      *error_message = EglErrorString("eglGetPlatformDisplayEXT(surfaceless)");
      return false;
    }
    EGLint major = 0;
    EGLint minor = 0;
    if (egl_api_.initialize(egl_display_, &major, &minor) != EGL_TRUE) {
      *error_message = EglErrorString("eglInitialize");
      DestroyEgl();
      return false;
    }
    egl_initialized_ = true;
    if (egl_api_.bind_api(EGL_OPENGL_API) != EGL_TRUE) {
      *error_message = EglErrorString("eglBindAPI(EGL_OPENGL_API)");
      DestroyEgl();
      return false;
    }

    const EGLint config_attributes[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_DEPTH_SIZE, 24,
        EGL_STENCIL_SIZE, 8,
        EGL_NONE};
    EGLConfig config = nullptr;
    EGLint config_count = 0;
    if (egl_api_.choose_config(egl_display_, config_attributes, &config, 1,
                               &config_count) != EGL_TRUE ||
        config_count != 1) {
      *error_message = EglErrorString("eglChooseConfig");
      DestroyEgl();
      return false;
    }

    egl_context_ = egl_api_.create_context(egl_display_, config,
                                           EGL_NO_CONTEXT, nullptr);
    if (egl_context_ == EGL_NO_CONTEXT) {
      *error_message = EglErrorString("eglCreateContext");
      DestroyEgl();
      return false;
    }

    if (egl_api_.make_current(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                              egl_context_) == EGL_TRUE) {
      backend_ = OffscreenBackend::kEglSurfaceless;
      return true;
    }
    egl_api_.get_error();

    const EGLint pbuffer_attributes[] = {
        EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    egl_surface_ = egl_api_.create_pbuffer_surface(
        egl_display_, config, pbuffer_attributes);
    if (egl_surface_ == EGL_NO_SURFACE) {
      *error_message = EglErrorString("eglCreatePbufferSurface");
      DestroyEgl();
      return false;
    }
    if (egl_api_.make_current(egl_display_, egl_surface_, egl_surface_,
                              egl_context_) != EGL_TRUE) {
      *error_message = EglErrorString("eglMakeCurrent(pbuffer)");
      DestroyEgl();
      return false;
    }
    backend_ = OffscreenBackend::kEglPbuffer;
    return true;
  }

  bool TryOsMesa(std::uint32_t width, std::uint32_t height,
                 std::string* error_message) {
    DestroyEgl();
    if (!LoadOsMesa(error_message)) {
      return false;
    }
    std::uint64_t pixel_count = 0;
    std::uint64_t buffer_bytes = 0;
    if (!detail::CheckedMultiply(width, height, &pixel_count) ||
        !detail::CheckedMultiply(pixel_count, 4, &buffer_bytes) ||
        buffer_bytes > protocol::kMaximumStreamPayloadBytes ||
        buffer_bytes > std::numeric_limits<std::size_t>::max()) {
      *error_message = "OSMesa backing buffer exceeds supported limits";
      return false;
    }

    osmesa_context_ =
        osmesa_api_.create_context(OSMESA_RGBA, 24, 8, 0, nullptr);
    if (osmesa_context_ == nullptr) {
      *error_message = "OSMesaCreateContextExt failed";
      return false;
    }
    osmesa_buffer_.resize(static_cast<std::size_t>(buffer_bytes));
    osmesa_width_ = static_cast<GLsizei>(width);
    osmesa_height_ = static_cast<GLsizei>(height);
    if (osmesa_api_.make_current(osmesa_context_, osmesa_buffer_.data(),
                                 GL_UNSIGNED_BYTE, osmesa_width_,
                                 osmesa_height_) != GL_TRUE) {
      *error_message = "OSMesaMakeCurrent failed";
      DestroyOsMesa();
      return false;
    }
    osmesa_api_.pixel_store(OSMESA_Y_UP, 1);
    backend_ = OffscreenBackend::kOsMesa;
    return true;
  }

  void DestroyEgl() {
    if (egl_display_ != EGL_NO_DISPLAY && egl_api_.make_current != nullptr) {
      egl_api_.make_current(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                            EGL_NO_CONTEXT);
      if (egl_surface_ != EGL_NO_SURFACE) {
        egl_api_.destroy_surface(egl_display_, egl_surface_);
      }
      if (egl_context_ != EGL_NO_CONTEXT) {
        egl_api_.destroy_context(egl_display_, egl_context_);
      }
      // MuJoCo caches OpenGL entry points process-wide, and surfaceless EGL
      // displays are commonly shared by all renderer contexts. Keep the
      // display initialized until process exit so one renderer cannot
      // invalidate another renderer or MuJoCo's dispatch table.
    }
    egl_display_ = EGL_NO_DISPLAY;
    egl_context_ = EGL_NO_CONTEXT;
    egl_surface_ = EGL_NO_SURFACE;
    egl_initialized_ = false;
    egl_api_ = {};
    if (egl_library_ != nullptr) {
      dlclose(egl_library_);
      egl_library_ = nullptr;
    }
  }

  void DestroyOsMesa() {
    if (osmesa_context_ != nullptr && osmesa_api_.destroy_context != nullptr) {
      osmesa_api_.destroy_context(osmesa_context_);
    }
    osmesa_context_ = nullptr;
    osmesa_buffer_.clear();
    osmesa_width_ = 0;
    osmesa_height_ = 0;
    osmesa_api_ = {};
    if (osmesa_library_ != nullptr) {
      dlclose(osmesa_library_);
      osmesa_library_ = nullptr;
    }
  }

  void Destroy() {
    DestroyEgl();
    DestroyOsMesa();
  }

  OffscreenBackend backend_ = OffscreenBackend::kOsMesa;
  EGLDisplay egl_display_ = EGL_NO_DISPLAY;
  EGLContext egl_context_ = EGL_NO_CONTEXT;
  EGLSurface egl_surface_ = EGL_NO_SURFACE;
  bool egl_initialized_ = false;
  void* egl_library_ = nullptr;
  EglApi egl_api_;
  OSMesaContext osmesa_context_ = nullptr;
  std::vector<unsigned char> osmesa_buffer_;
  GLsizei osmesa_width_ = 0;
  GLsizei osmesa_height_ = 0;
  void* osmesa_library_ = nullptr;
  OsMesaApi osmesa_api_;
};

bool WantsFormat(const CameraProfile& profile,
                 mujoco_sim::v3::StreamFormat format) {
  return std::any_of(profile.outputs.begin(), profile.outputs.end(),
                     [format](const CameraOutput& output) {
                       return output.format == format;
                     });
}

std::vector<std::byte> FlipRgbTopLeft(const std::vector<unsigned char>& rgb,
                                      std::uint32_t width,
                                      std::uint32_t height) {
  const std::size_t stride = static_cast<std::size_t>(width) * 3U;
  std::vector<std::byte> output(rgb.size());
  for (std::size_t row = 0; row < height; ++row) {
    const std::size_t source_row = static_cast<std::size_t>(height) - row - 1U;
    std::memcpy(output.data() + row * stride,
                rgb.data() + source_row * stride, stride);
  }
  return output;
}

std::vector<float> MetricDepthTopLeft(const std::vector<float>& depth,
                                      std::uint32_t width,
                                      std::uint32_t height, double near_plane,
                                      double far_plane, int depth_map) {
  std::vector<float> output(depth.size());
  for (std::size_t row = 0; row < height; ++row) {
    const std::size_t source_row = static_cast<std::size_t>(height) - row - 1U;
    for (std::size_t column = 0; column < width; ++column) {
      const double sample = std::clamp(
          static_cast<double>(depth[source_row * width + column]), 0.0, 1.0);
      double denominator = 0.0;
      if (depth_map == mjDEPTH_ZEROFAR) {
        denominator = near_plane + sample * (far_plane - near_plane);
      } else {
        denominator = far_plane - sample * (far_plane - near_plane);
      }
      output[row * width + column] = static_cast<float>(
          near_plane * far_plane / denominator);
    }
  }
  return output;
}

std::vector<float> MakePointCloud(
    std::span<const float> depth, std::uint32_t width, std::uint32_t height,
    const mujoco_sim::v3::CameraIntrinsics& intrinsics) {
  std::vector<float> points(depth.size() * 3U);
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t column = 0; column < width; ++column) {
      const std::size_t index = row * width + column;
      const double z = depth[index];
      points[3 * index] = static_cast<float>(
          (static_cast<double>(column) - intrinsics.cx()) * z /
          intrinsics.fx());
      points[3 * index + 1] = static_cast<float>(
          (static_cast<double>(row) - intrinsics.cy()) * z /
          intrinsics.fy());
      points[3 * index + 2] = static_cast<float>(z);
    }
  }
  return points;
}

std::vector<std::uint32_t> MakeSegmentation(
    const mjModel* model, mjData* data, const mjvOption& option,
    const mjvScene& scene, std::uint32_t width, std::uint32_t height) {
  const std::size_t pixel_count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<std::uint32_t> labels(pixel_count, 0U);
  constexpr std::size_t kRayBatchSize = 65536;
  std::vector<mjtNum> directions(3U * std::min(pixel_count, kRayBatchSize));
  std::vector<mjtNum> distances(std::min(pixel_count, kRayBatchSize));
  std::vector<int> geom_ids(std::min(pixel_count, kRayBatchSize));
  std::vector<double> forward_cosines(
      std::min(pixel_count, kRayBatchSize));

  const mjvGLCamera camera =
      mjv_averageCamera(&scene.camera[0], &scene.camera[1]);
  const double vertical_span = camera.frustum_top - camera.frustum_bottom;
  const double horizontal_span = camera.frustum_width > 0.0F
                                     ? 2.0 * camera.frustum_width
                                     : vertical_span *
                                           static_cast<double>(width) /
                                           static_cast<double>(height);
  const double left = camera.frustum_center - 0.5 * horizontal_span;
  const double near_plane = camera.frustum_near;
  const double far_plane = camera.frustum_far;
  const std::array<double, 3> right = {
      camera.forward[1] * camera.up[2] -
          camera.forward[2] * camera.up[1],
      camera.forward[2] * camera.up[0] -
          camera.forward[0] * camera.up[2],
      camera.forward[0] * camera.up[1] -
          camera.forward[1] * camera.up[0]};
  const mjtNum ray_origin[3] = {camera.pos[0], camera.pos[1], camera.pos[2]};

  for (std::size_t start = 0; start < pixel_count;
       start += kRayBatchSize) {
    const std::size_t count =
        std::min(kRayBatchSize, pixel_count - start);
    double ray_cutoff = far_plane;
    for (std::size_t offset = 0; offset < count; ++offset) {
      const std::size_t index = start + offset;
      const std::size_t row = index / width;
      const std::size_t column = index % width;
      const double x =
          left + (static_cast<double>(column) + 0.5) * horizontal_span /
                     static_cast<double>(width);
      const double y =
          camera.frustum_top -
          (static_cast<double>(row) + 0.5) * vertical_span /
              static_cast<double>(height);
      double norm_squared = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        const double component = camera.forward[axis] * near_plane +
                                 right[axis] * x + camera.up[axis] * y;
        directions[3 * offset + axis] = component;
        norm_squared += component * component;
      }
      const double inverse_norm = 1.0 / std::sqrt(norm_squared);
      for (int axis = 0; axis < 3; ++axis) {
        directions[3 * offset + axis] *= inverse_norm;
      }
      double forward_cosine = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        forward_cosine +=
            directions[3 * offset + axis] * camera.forward[axis];
      }
      forward_cosines[offset] = forward_cosine;
      if (forward_cosine > 0.0) {
        ray_cutoff = std::max(ray_cutoff, far_plane / forward_cosine);
      }
    }

    mj_multiRay(model, data, ray_origin, directions.data(), option.geomgroup,
                1, -1, geom_ids.data(), distances.data(), nullptr,
                static_cast<int>(count), ray_cutoff);
    for (std::size_t offset = 0; offset < count; ++offset) {
      const double optical_depth =
          distances[offset] * forward_cosines[offset];
      if (geom_ids[offset] >= 0 && forward_cosines[offset] > 0.0 &&
          optical_depth >= near_plane && optical_depth <= far_plane) {
        labels[start + offset] =
            static_cast<std::uint32_t>(geom_ids[offset]) + 1U;
      }
    }
  }
  return labels;
}

}  // namespace

struct OffscreenRenderer::Impl {
  ~Impl() {
    std::string ignored;
    if (render_context_initialized && native_context.MakeCurrent(&ignored)) {
      mjr_freeContext(&render_context);
    }
    if (scene_initialized) {
      mjv_freeScene(&scene);
    }
  }

  CameraProfile profile;
  std::vector<mujoco_sim::v3::StreamDescriptor> descriptors;
  const mjModel* model = nullptr;
  int camera_id = -1;
  NativeContext native_context;
  mjvOption option{};
  mjvCamera camera{};
  mjvScene scene{};
  mjrContext render_context{};
  bool scene_initialized = false;
  bool render_context_initialized = false;
};

std::string_view ToString(OffscreenBackend backend) noexcept {
  switch (backend) {
    case OffscreenBackend::kEglSurfaceless:
      return "egl-surfaceless";
    case OffscreenBackend::kEglPbuffer:
      return "egl-pbuffer";
    case OffscreenBackend::kOsMesa:
      return "osmesa";
  }
  return "unknown";
}

OffscreenRenderer::OffscreenRenderer(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
OffscreenRenderer::~OffscreenRenderer() = default;
OffscreenRenderer::OffscreenRenderer(OffscreenRenderer&&) noexcept = default;
OffscreenRenderer& OffscreenRenderer::operator=(
    OffscreenRenderer&&) noexcept = default;

std::unique_ptr<OffscreenRenderer> OffscreenRenderer::Create(
    const core::SimulationSnapshot& snapshot, CameraProfile profile,
    OffscreenRendererOptions options, std::string* error_message) {
  detail::ClearError(error_message);
  std::unique_lock platform_lock(g_mujoco_platform_mutex);
  auto impl = std::make_unique<Impl>();
  impl->profile = std::move(profile);
  if (!detail::BuildCameraDescriptors(snapshot, impl->profile,
                                      &impl->descriptors, &impl->camera_id,
                                      error_message)) {
    return nullptr;
  }

  detail::StreamShape context_shape;
  if (!detail::StreamShapeFor(mujoco_sim::v3::DEPTH32F,
                              impl->profile.width, impl->profile.height,
                              &context_shape, error_message)) {
    return nullptr;
  }
  BackendPreference effective_backend = BackendPreference::kAutomatic;
  if (!ResolveBackendPreference(options.backend, &effective_backend,
                                error_message)) {
    return nullptr;
  }
  if (!impl->native_context.Initialize(impl->profile.width,
                                       impl->profile.height, effective_backend,
                                       error_message)) {
    return nullptr;
  }
  if (!impl->native_context.MakeCurrent(error_message)) {
    return nullptr;
  }

  impl->model = snapshot.model();
  std::int64_t requested_capacity =
      static_cast<std::int64_t>(impl->model->ngeom) + impl->model->nsite +
      impl->model->nbody + 100;
  if (requested_capacity > std::numeric_limits<int>::max()) {
    detail::Fail("model is too large for a MuJoCo render scene",
                 error_message);
    return nullptr;
  }
  const int scene_capacity =
      std::max(1000, static_cast<int>(requested_capacity));
  mjv_defaultOption(&impl->option);
  mjv_defaultCamera(&impl->camera);
  mjv_defaultScene(&impl->scene);
  mjr_defaultContext(&impl->render_context);
  mjv_makeScene(impl->model, &impl->scene, scene_capacity);
  impl->scene_initialized = true;
  mjr_makeContext(impl->model, &impl->render_context, mjFONTSCALE_100);
  impl->render_context_initialized = true;
  g_mujoco_platform = FamilyOf(impl->native_context.backend());
  impl->native_context.RetainDispatchLibraryForProcess(
      &g_mujoco_platform_library);
  mjr_resizeOffscreen(static_cast<int>(impl->profile.width),
                      static_cast<int>(impl->profile.height),
                      &impl->render_context);
  mjr_setBuffer(mjFB_OFFSCREEN, &impl->render_context);
  const mjrRect maximum = mjr_maxViewport(&impl->render_context);
  if (impl->render_context.currentBuffer != mjFB_OFFSCREEN ||
      maximum.width < static_cast<int>(impl->profile.width) ||
      maximum.height < static_cast<int>(impl->profile.height)) {
    detail::Fail("MuJoCo did not create the requested offscreen framebuffer",
                 error_message);
    return nullptr;
  }

  return std::unique_ptr<OffscreenRenderer>(
      new OffscreenRenderer(std::move(impl)));
}

const CameraProfile& OffscreenRenderer::profile() const noexcept {
  return impl_->profile;
}

const std::vector<mujoco_sim::v3::StreamDescriptor>&
OffscreenRenderer::descriptors() const noexcept {
  return impl_->descriptors;
}

OffscreenBackend OffscreenRenderer::backend() const noexcept {
  return impl_->native_context.backend();
}

bool OffscreenRenderer::Capture(const core::SimulationSnapshot& snapshot,
                                const CaptureMetadata& metadata,
                                std::vector<RawFrame>* frames,
                                std::string* error_message) {
  detail::ClearError(error_message);
  if (frames == nullptr) {
    return detail::Fail("camera frame output is null", error_message);
  }
  frames->clear();
  if (!detail::ValidateCaptureMetadata(metadata, error_message)) {
    return false;
  }
  if (snapshot.model() != impl_->model || snapshot.data() == nullptr) {
    return detail::Fail(
        "camera snapshot is not the model used to create this renderer",
        error_message);
  }
  if (!impl_->native_context.MakeCurrent(error_message)) {
    return false;
  }

  impl_->camera.type = mjCAMERA_FIXED;
  impl_->camera.fixedcamid = impl_->camera_id;
  mjv_updateScene(impl_->model, snapshot.data(), &impl_->option, nullptr,
                  &impl_->camera, mjCAT_ALL, &impl_->scene);
  impl_->scene.stereo = mjSTEREO_NONE;
  impl_->scene.flags[mjRND_SEGMENT] = 0;
  impl_->scene.flags[mjRND_IDCOLOR] = 0;
  mjr_setBuffer(mjFB_OFFSCREEN, &impl_->render_context);
  const mjrRect viewport{0, 0, static_cast<int>(impl_->profile.width),
                         static_cast<int>(impl_->profile.height)};
  mjr_render(viewport, &impl_->scene, &impl_->render_context);

  const std::size_t pixel_count =
      static_cast<std::size_t>(impl_->profile.width) * impl_->profile.height;
  const bool wants_rgb = WantsFormat(impl_->profile, mujoco_sim::v3::RGB8);
  const bool wants_depth =
      WantsFormat(impl_->profile, mujoco_sim::v3::DEPTH32F) ||
      WantsFormat(impl_->profile, mujoco_sim::v3::XYZ32F);
  std::vector<unsigned char> rgb(wants_rgb ? 3U * pixel_count : 0U);
  std::vector<float> raw_depth(wants_depth ? pixel_count : 0U);
  if (wants_rgb || wants_depth) {
    mjr_readPixels(wants_rgb ? rgb.data() : nullptr,
                   wants_depth ? raw_depth.data() : nullptr, viewport,
                   &impl_->render_context);
  }
  const int gl_error = mjr_getError();
  if (gl_error != 0) {
    return detail::Fail("MuJoCo rendering reported OpenGL error " +
                            std::to_string(gl_error),
                        error_message);
  }

  std::vector<std::byte> rgb_payload;
  if (wants_rgb) {
    rgb_payload = FlipRgbTopLeft(rgb, impl_->profile.width,
                                 impl_->profile.height);
  }
  std::vector<float> metric_depth;
  if (wants_depth) {
    const mjvGLCamera camera = mjv_averageCamera(
        &impl_->scene.camera[0], &impl_->scene.camera[1]);
    const double near_plane = camera.frustum_near;
    const double far_plane = camera.frustum_far;
    if (!std::isfinite(near_plane) || !std::isfinite(far_plane) ||
        near_plane <= 0.0 || far_plane <= near_plane) {
      return detail::Fail("MuJoCo camera clipping planes are invalid",
                          error_message);
    }
    metric_depth = MetricDepthTopLeft(
        raw_depth, impl_->profile.width, impl_->profile.height, near_plane,
        far_plane, impl_->render_context.readDepthMap);
  }

  std::vector<std::uint32_t> segmentation;
  if (WantsFormat(impl_->profile, mujoco_sim::v3::SEGMENTATION32U)) {
    segmentation = MakeSegmentation(
        impl_->model, snapshot.data(), impl_->option, impl_->scene,
        impl_->profile.width, impl_->profile.height);
  }

  const auto& intrinsics = impl_->descriptors.front().camera_intrinsics();
  std::vector<float> point_cloud;
  if (WantsFormat(impl_->profile, mujoco_sim::v3::XYZ32F)) {
    point_cloud = MakePointCloud(metric_depth, impl_->profile.width,
                                 impl_->profile.height, intrinsics);
  }

  frames->reserve(impl_->descriptors.size());
  for (const auto& descriptor : impl_->descriptors) {
    std::vector<std::byte> payload;
    switch (descriptor.format()) {
      case mujoco_sim::v3::RGB8:
        payload = rgb_payload;
        break;
      case mujoco_sim::v3::DEPTH32F:
        payload = detail::ToBytes<float>(metric_depth);
        break;
      case mujoco_sim::v3::SEGMENTATION32U:
        payload = detail::ToBytes<std::uint32_t>(segmentation);
        break;
      case mujoco_sim::v3::XYZ32F:
        payload = detail::ToBytes<float>(point_cloud);
        break;
      default:
        return detail::Fail("validated camera format changed unexpectedly",
                            error_message);
    }
    if (payload.size() != descriptor.maximum_payload_bytes()) {
      frames->clear();
      return detail::Fail("camera payload shape changed unexpectedly",
                          error_message);
    }
    frames->push_back(
        detail::MakeRawFrame(descriptor, metadata, std::move(payload)));
  }
  return true;
}

}  // namespace mujoco_sim::sensor
