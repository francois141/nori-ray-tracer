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
#include <nori/blockedarray.h>

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

/**
 * \brief Image texture
 */
template <typename Tmemory, typename Treturn>
class ImageTexture : public Texture<Treturn> {
public:

    ImageTexture(const PropertyList &props) {
        /* Lookup parameters */
        m_filename = props.getString("fileName", "");
        m_doTri = props.getBoolean("DoTrilinear", false);
        m_maxAniso = props.getFloat("AnisotropyThreshold", 0.0f);
        m_scale = props.getFloat("Scale", 0.0f);
        m_gamma = props.getBoolean("Gamma", false);
    }

    virtual Treturn eval(const Point2f & uv) override {
        
    }

private:
    const std::string & m_filename;
    bool m_doTri; //Trilinear filtering flag
    float m_maxAniso; //Max anisotropy threshold
    float m_scale;
    bool m_gamma;

};

NORI_NAMESPACE_END

