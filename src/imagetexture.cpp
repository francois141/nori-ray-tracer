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

NORI_NAMESPACE_BEGIN

/**
 * \brief Superclass of all texture
 */
template <typename Tmemory, typename Treturn>
class ImageTexture : public Texture {
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

