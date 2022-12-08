/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
#include <nori/texture.h>
#include <nori/common.h>
#include <stb_image.h>

NORI_NAMESPACE_BEGIN

/**
 * @brief Enum to define how textures are repeated when mapped
 * to a specific geometry
 */
enum class ImageWrap { Repeat, Black, Clamp };

/**
 * @brief Defines a mipmap, used to improve the filtering of a texture
 * 
 * @tparam T the return type of said texture's eval method
 */
/*template<typename T>
class MipMap {
public:
    MipMap(const Point2i &resolution, const T* data,
            bool doTri = false, float maxAniso = 8.f, 
            ImageWrap wrapMode = ImageWrap::Repeat) : 
        
private:
    const bool m_doTri;
    const float m_maxAniso;
    const ImageWrap m_wrapMode;
    Point2i m_resolution;
    std::vector<std::unique_ptr<BlockedArray<T>>> pyramid;
    
};*/

ImageTexture::ImageTexture(const PropertyList &props) : m_filename(props.getString("fileName", "")) {
    // Load in image
    m_data = stbi_load(
        m_filename.c_str(), 
        &m_width, 
        &m_height, 
        &m_channels, 
        STBI_rgb
    );
}

#define RED_CHANNEL 0
#define GREEN_CHANNEL 1
#define BLUE_CHANNEL 2
#define ALPHA_CHANNEL 3

// Get data from texel coordinates
Color3f ImageTexture::getData(const Point2f & xy) const  {
    // Clamp the texel coordinates to be within the texture's range
    int x = clamp(static_cast<int>(xy.x()), 0, m_width - 1);
    int y = clamp(static_cast<int>(xy.y()), 0, m_height - 1);

    // Retrieve the data for the individual color channels
    float redData = static_cast<float>(m_data[(x + m_width * y) * STBI_rgb + RED_CHANNEL]) / UCHAR_MAX;
    float greenData = static_cast<float>(m_data[(x + m_width * y) * STBI_rgb + GREEN_CHANNEL]) / UCHAR_MAX;
    float blueData = static_cast<float>(m_data[(x + m_width * y) * STBI_rgb + BLUE_CHANNEL]) / UCHAR_MAX;

    return Color3f(redData, greenData, blueData);
}

Color3f ImageTexture::eval(const Point2f & uv) {
    // Retrieve texel coordinates from uv coordinates
    float x(1.0f - uv.x() * m_width), y(uv.y() * m_height);

    // Retrieve data values of neighboring pixels for bilinear interpolation
    Color3f v00(getData(Point2f(x, y))),
            v01(getData(Point2f(x, y + 1.0f))),
            v10(getData(Point2f(x + 1.0f, y))),
            v11(getData(Point2f(x + 1.0f, y + 1.0f)));

    // Compute bilinear interpolation (as done in PBRT)
    float dstdx = (1.0f - uv.x()) * m_width - x;
    float dstdy = uv.y() * m_height - y;

    return (1.0f - dstdx) * (1.0f - dstdy) * v00 +
            (1.0f - dstdx) * dstdy * v01 +
            dstdx * (1.0f - dstdy) * v10 +
            dstdx * dstdy * v11;
}

std::string ImageTexture::toString() const {
    return tfm::format(
        "ImageTexture[\n"
                "  filename = %s,\n"
                "]",
        m_filename
    );
}
NORI_NAMESPACE_END

