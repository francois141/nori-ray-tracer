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

#if !defined(__NORI_TEXTURE_H)
#define __NORI_TEXTURE_H

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Superclass of all texture
 */
template <typename T>
class Texture : public NoriObject {
public:
    Texture() {}
    virtual ~Texture() {}

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    virtual EClassType getClassType() const override { return ETexture; }

    virtual T eval(const Point2f & uv) = 0;
};

/**
 * \brief Image texture
 */
class ImageTexture : public Texture<Color3f> {
public:

    ImageTexture(const PropertyList &props) : m_filename(props.getString("fileName", "")) { }

    Color3f eval(const Point2f & uv) override;

    std::string toString() const override;

private:
    const std::string & m_filename;
    uint8_t* m_data;
    int m_width;
    int m_height;
    int m_channels; // RGB or RGBA

    Color3f getData(const Point2f & uv) const;
};

NORI_REGISTER_CLASS(ImageTexture, "ImageTexture");

NORI_NAMESPACE_END

#endif /* __NORI_TEXTURE_H */
