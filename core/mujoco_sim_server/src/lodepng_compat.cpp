#include "lodepng.h"

#include <png.h>

namespace lodepng {

unsigned encode(const std::string& filename, const unsigned char* image,
                unsigned width, unsigned height,
                LodePNGColorType color_type) {
  if (image == nullptr || width == 0 || height == 0 ||
      color_type != LCT_RGB || width > PNG_UINT_31_MAX ||
      height > PNG_UINT_31_MAX) {
    return 1;
  }

  png_image output{};
  output.version = PNG_IMAGE_VERSION;
  output.width = width;
  output.height = height;
  output.format = PNG_FORMAT_RGB;

  const int written = png_image_write_to_file(
      &output, filename.c_str(), 0, image, 0, nullptr);
  png_image_free(&output);
  return written == 0 ? 2 : 0;
}

}  // namespace lodepng
