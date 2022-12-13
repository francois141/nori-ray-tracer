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
#include <filesystem/resolver.h>

NORI_NAMESPACE_BEGIN

/**
 * @brief Enum to define how textures are repeated when mapped
 * to a specific geometry
 */
enum class ImageWrap { Repeat, Clamp };
ImageWrap wrapTypeFromString(std::string type_name) {
    if(type_name == "repeat") {
        return ImageWrap::Repeat;
    }
    if(type_name == "clamp") {
        return ImageWrap::Clamp;
    }
    // Invalid name was given
    throw NoriException("Invalid wrap type name %s", type_name);
}
std::string wrapToString(ImageWrap iw) {
    switch (iw) {
    case ImageWrap::Repeat:
        return "repeat";
    case ImageWrap::Clamp:
        return "clamp";
    
    default:
        return "no wrap";
    }
}

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

/**
 * \brief Image texture
 */
class ImageTexture : public Texture<Color3f> {
public:

    ImageTexture(const PropertyList &props);

    Color3f eval(const Point2f & uv) const override;

    std::string toString() const override;

private:
    std::string m_filename;
    ImageWrap m_wrap; // How is the texture treated at image edge
    uint8_t* m_data;
    int m_width;
    int m_height;
    int m_channels; // RGB or RGBA

    Color3f getData(const Point2f & uv) const;
};

NORI_REGISTER_CLASS(ImageTexture, "ImageTexture");

ImageTexture::ImageTexture(const PropertyList &props) {
    m_filename = props.getString("fileName", "textures/default.png");
    m_wrap = wrapTypeFromString(props.getString("wrap", "repeat"));

    if(!m_filename.empty()) {
        filesystem::path file_path = getFileResolver()->resolve(m_filename);
         // Load in image: For some reason this doesn't load in anything...
        m_data = stbi_load(
            file_path.str().c_str(), 
            &m_width, 
            &m_height, 
            &m_channels, 
            STBI_rgb
        );
    }

    if(!m_data) {
        throw NoriException("No image data was loaded!");
    }
}

#define RED_CHANNEL 0
#define GREEN_CHANNEL 1
#define BLUE_CHANNEL 2
#define ALPHA_CHANNEL 3

// Get data from texel coordinates
Color3f ImageTexture::getData(const Point2f & xy) const  {
    // Clamp or mod the texel coordinates to be within the texture's range
    int x, y;
    if(m_wrap == ImageWrap::Repeat) {
        x = (static_cast<int>(xy.x()) % m_width);
        y = (static_cast<int>(xy.y()) % m_height);
    } else {
        x = clamp(static_cast<int>(xy.x()), 0, m_width - 1);
        y = clamp(static_cast<int>(xy.y()), 0, m_height - 1);
    }

    // Retrieve the data for the individual color channels
    float redData = static_cast<float>(m_data[((x + m_width * y) * STBI_rgb + RED_CHANNEL)]) / UCHAR_MAX;
    float greenData = static_cast<float>(m_data[((x + m_width * y) * STBI_rgb + GREEN_CHANNEL)]) / UCHAR_MAX;
    float blueData = static_cast<float>(m_data[((x + m_width * y) * STBI_rgb + BLUE_CHANNEL)]) / UCHAR_MAX;

    return Color3f(redData, greenData, blueData);
}

Color3f ImageTexture::eval(const Point2f & uv) const {
    // Retrieve texel coordinates from uv coordinates
    float x(uv.x() * m_width), y(uv.y() * m_height);

    // Retrieve data values of neighboring pixels for bilinear interpolation
    Color3f v00(getData(Point2f(x, y))),
            v01(getData(Point2f(x, y + 1.0f))),
            v10(getData(Point2f(x + 1.0f, y))),
            v11(getData(Point2f(x + 1.0f, y + 1.0f)));

    // Compute bilinear interpolation (as done in PBRT)
    float dstdx = uv.x() * m_width - x;
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
                "  wrap = %s\n"
                "]",
        m_filename, 
        wrapToString(m_wrap)
    );
}
NORI_NAMESPACE_END

